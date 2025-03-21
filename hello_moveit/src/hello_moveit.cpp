// just position code

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <franka_msgs/action/homing.hpp>
#include <iostream>
#include <future>
#include <chrono>
#include <typeinfo>

#include <zed_msgs/msg/objects_stamped.hpp>
#include <Eigen/Dense>

class GripperControlNode : public rclcpp::Node
{
public:
    using Grasp = franka_msgs::action::Grasp;
    using Homing = franka_msgs::action::Homing;
    using Move = franka_msgs::action::Move;
    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;
    using GoalHandleHoming = rclcpp_action::ClientGoalHandle<Homing>;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    GripperControlNode() : Node("gripper_client")
    {
        grasp_client_ = rclcpp_action::create_client<Grasp>(this, "/panda_gripper/grasp");
        homing_client_ = rclcpp_action::create_client<Homing>(this, "/panda_gripper/homing");
        move_client_ = rclcpp_action::create_client<Move>(this, "/panda_gripper/move");
    }

    void sendGraspCommand(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
    {
        if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Grasp action server not available!");
            return;
        }

        auto goal_msg = Grasp::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;
        goal_msg.force = force;
        goal_msg.epsilon.inner = epsilon_inner;
        goal_msg.epsilon.outer = epsilon_outer;

        auto send_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
        send_goal_options.result_callback =
            [](const GoalHandleGrasp::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("GripperControlNode"), "Grasp succeeded");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperControlNode"), "Grasp failed");
            }
        };

        grasp_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void sendHomingCommand()
    {
        if (!homing_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Homing action server not available!");
            return;
        }

        auto goal_msg = Homing::Goal();

        auto send_goal_options = rclcpp_action::Client<Homing>::SendGoalOptions();
        send_goal_options.result_callback =
            [](const GoalHandleHoming::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("GripperControlNode"), "Homing succeeded");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperControlNode"), "Homing failed");
            }
        };

        homing_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void sendMoveCommand(double width, double speed)
    {
        if (!move_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Move action server not available!");
            return;
        }

        auto goal_msg = Move::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;

        auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
        send_goal_options.result_callback =
            [](const GoalHandleMove::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(rclcpp::get_logger("GripperControlNode"), "Move succeeded");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("GripperControlNode"), "Move failed");
            }
        };

        move_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
    rclcpp_action::Client<Homing>::SharedPtr homing_client_;
    rclcpp_action::Client<Move>::SharedPtr move_client_;
};

// Define a struct to hold object data (ID, label, and position)
struct ObjectData
{
    int id;                        // Object ID
    std::string label;             // Object label
    std::array<float, 3> position; // Object position (x, y, z)
};

class ObjectDetectionSubscriber : public rclcpp::Node
{
public:
    ObjectDetectionSubscriber() : Node("zed_object_subscriber")
    {
        subscription_ = this->create_subscription<zed_msgs::msg::ObjectsStamped>(
            "/zed/zed_node/obj_det/objects", 10,
            std::bind(&ObjectDetectionSubscriber::objectsCallback, this, std::placeholders::_1));
    }

    // Getter function to retrieve the latest detected objects
    std::vector<ObjectData> getDetectedObjects()
    {
        return detected_objects_; // Return a copy of the detected objects
    }

private:
    std::vector<ObjectData> detected_objects_; // Stores detected objects

    rclcpp::Subscription<zed_msgs::msg::ObjectsStamped>::SharedPtr subscription_;

    // Callback function to store detected objects
    void objectsCallback(const zed_msgs::msg::ObjectsStamped::SharedPtr msg)
    {
        detected_objects_.clear(); // Clear previous data

        for (const auto &obj : msg->objects)
        {
            ObjectData object_data;
            object_data.id = obj.label_id;
            object_data.label = obj.label;
            object_data.position = {obj.position[0], obj.position[1], obj.position[2]};

            detected_objects_.push_back(object_data);
        }

        // RCLCPP_INFO(this->get_logger(), "Stored %ld objects", detected_objects_.size());
    }
};

void moveArmToPose(
    moveit::planning_interface::MoveGroupInterface &move_group_interface_arm,
    const geometry_msgs::msg::Pose &target_pose,
    rclcpp::Logger logger)
{

    move_group_interface_arm.setPoseTarget(target_pose);

    // Create a plan to the target pose
    auto const [success, plan] = [&move_group_interface_arm]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface_arm.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan if successful
    if (success)
    {
        move_group_interface_arm.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning failed!");
    }
}

void moveArmToJointPositions(
    moveit::planning_interface::MoveGroupInterface &move_group_interface_arm,
    const std::vector<double> &joint_positions,
    rclcpp::Logger logger)
{
    // Set the target joint values
    move_group_interface_arm.setJointValueTarget(joint_positions);

    // Create a plan to the target joint values
    auto const [success, plan] = [&move_group_interface_arm]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface_arm.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan if successful
    if (success)
    {
        move_group_interface_arm.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning to joint positions failed!");
    }
}

// void performTask(moveit::planning_interface::MoveGroupInterface &move_group_interface_arm,
//                  std::array<float, 3> position,
//                  rclcpp::Logger logger,
//                  const std::shared_ptr<rclcpp::Node> &gripper_node)
// {
//     // Move to the ball
//     moveArmToPose(move_group_interface_arm, target_pose, logger);
//     rclcpp::sleep_for(std::chrono::seconds(15));

//     // Grab the ball
//     gripper_node->sendGraspCommand(0.025, 0.03, 105, 0.05, 0.05);
//     rclcpp::sleep_for(std::chrono::seconds(5));

//     // Move back to the ready position to make sure transformation to cup is correct
//     moveArmToPose(move_group_interface_arm, ready_pose, logger);
//     rclcpp::sleep_for(std::chrono::seconds(15));

//     // Move to the cup to deposit the ball
//     moveArmToPose(move_group_interface_arm, deposit_pose, logger);
//     rclcpp::sleep_for(std::chrono::seconds(15));

//     // Release the ball
//     gripper_node->sendMoveCommand(0.08, 0.03);
//     rclcpp::sleep_for(std::chrono::seconds(5));

//     // Move back to the ready position to get ready for the next ball
//     moveArmToPose(move_group_interface_arm, ready_pose, logger);
//     rclcpp::sleep_for(std::chrono::seconds(15));
// }

std::string getUserInputWithTimeout(int timeout_ms)
{
    std::promise<std::string> promise;
    std::future<std::string> future = promise.get_future();

    // Start a thread to get user input
    std::thread inputThread([&promise]()
                            {
                                std::string input;
                                std::getline(std::cin, input);
                                promise.set_value(input); // Send input to future
                            });

    // Wait for input or timeout
    if (future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready)
    {
        inputThread.join();  // Ensure thread exits before returning
        return future.get(); // Get the user input
    }
    else
    {
        std::cout << "Timeout! No input received.\n";
        inputThread.detach(); // Detach thread to prevent blocking
        return "";            // Return empty string if timeout occurs
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create arm and gripper nodes
    auto const arm_node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto gripper_node = std::make_shared<GripperControlNode>();

    auto object_node = std::make_shared<ObjectDetectionSubscriber>();

    // Create the MoveIt MoveGroup Interface for Arm
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface_arm = MoveGroupInterface(arm_node, "panda_arm");
    move_group_interface_arm.setPlanningTime(10);

    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();

    collision_object.id = "box1";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    primitive.dimensions[primitive.BOX_Z] = 0.17;

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // RCLCPP_INFO(logger, "Add an object into the world");
    // planning_scene_interface.addCollisionObjects(collision_objects);

    // visual_tools.publishText(text_pose, "Add_object", rvt::WHITE, rvt::XLARGE);
    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

    // Create some saved positions to use later
    // Set a target Pose (pickup the ball)
    auto const target_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 0.74868;
        msg.orientation.y = 0.66245;
        msg.orientation.z = 0.021153;
        msg.orientation.w = 0.013489;
        msg.position.x = 0.3529; // from vision
        msg.position.y = 0.4230; // from vision;
        msg.position.z = 0.17181293666362762;
        return msg;
    }();

    // Set a ready pose
    auto const ready_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 0.92178;
        msg.orientation.y = -0.3877;
        msg.orientation.z = 0.0020461;
        msg.orientation.w = 0.0016923;
        msg.position.x = 0.30713;
        msg.position.y = -0.00099106;
        msg.position.z = 0.59015;
        return msg;
    }();

    // Set a deposit pose (drop the ball)
    auto const deposit_pose = []
    {
        geometry_msgs::msg::Pose msg;
        msg.orientation.x = 0.80561;
        msg.orientation.y = -0.59233;
        msg.orientation.z = 0.0078321;
        msg.orientation.w = 0.0091214;
        msg.position.x = 0.44129;
        msg.position.y = -0.21454;
        msg.position.z = 0.29234;
        return msg;
    }();

    // Home the gripper (calibration)
    gripper_node->sendHomingCommand();
    rclcpp::sleep_for(std::chrono::seconds(10));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_node);
    executor.add_node(gripper_node);
    executor.add_node(object_node);

    // Run executor in a separate thread
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    Eigen::Matrix4f T1_0;
    T1_0 << 0.0, 1.0, 0.0, 0.2016,
        -0.9511, 0.0, -0.3090, 1.0604,
        -0.3090, 0.0, 0.9511, 0.3429,
        0.0, 0.0, 0.0, 1.0;

    // std::cout << "Pick and Place task started... " << std::endl;

    // // Move to the ball
    // moveArmToPose(move_group_interface_arm, target_pose, logger);
    // rclcpp::sleep_for(std::chrono::seconds(10));

    // // Grab the ball
    // gripper_node->sendGraspCommand(0.025, 0.03, 105, 0.05, 0.05);
    // rclcpp::sleep_for(std::chrono::seconds(5));

    // // Move back to the ready position to make sure transformation to cup is correct
    // moveArmToPose(move_group_interface_arm, ready_pose, logger);
    // rclcpp::sleep_for(std::chrono::seconds(10));

    // // Move to the cup to deposit the ball
    // moveArmToPose(move_group_interface_arm, deposit_pose, logger);
    // rclcpp::sleep_for(std::chrono::seconds(10));

    // // Release the ball
    // gripper_node->sendMoveCommand(0.08, 0.03);
    // rclcpp::sleep_for(std::chrono::seconds(5));

    // // Move back to the ready position to get ready for the next ball
    // moveArmToPose(move_group_interface_arm, ready_pose, logger);
    // rclcpp::sleep_for(std::chrono::seconds(10));

    while (rclcpp::ok())
    {
        auto objects = object_node->getDetectedObjects();

        for (const auto &obj : objects)
        {
            RCLCPP_INFO(arm_node->get_logger(),
                        "Object: %s (ID: %d) - Position: x=%.2f, y=%.2f, z=%.2f",
                        obj.label.c_str(), obj.id, obj.position[0], obj.position[1], obj.position[2]);

            if (obj.label != "SPORT")
                continue;

            Eigen::Vector4f obj_pos_cam_frame(static_cast<float>(obj.position[0]),
                                              static_cast<float>(obj.position[1]),
                                              static_cast<float>(obj.position[2]),
                                              1.0);

            Eigen::Vector4f obj_pos_base_frame = T1_0 * obj_pos_cam_frame;

            float new_x = obj_pos_base_frame(0);
            float new_y = obj_pos_base_frame(1);

            // Set a target Pose (pickup the ball)
            geometry_msgs::msg::Pose new_target_pose;
            new_target_pose.orientation.x = 0.74868;
            new_target_pose.orientation.y = 0.66245;
            new_target_pose.orientation.z = 0.021153;
            new_target_pose.orientation.w = 0.013489;
            new_target_pose.position.x = new_x; // from vision
            new_target_pose.position.y = new_y; // from vision;
            new_target_pose.position.z = 0.17181293666362762;

            std::cout << "Resulting vector:\n"
                      << obj_pos_base_frame << std::endl;

            // std::cout << "Please enter any key to start the Pick and Place task... ";
            // std::string user_input = getUserInputWithTimeout(5000);

            // if (!user_input.empty())
            // {
            std::cout << "Pick and Place task started... " << std::endl;

            // Move to the ball
            moveArmToPose(move_group_interface_arm, new_target_pose, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));

            // Grab the ball
            gripper_node->sendGraspCommand(0.025, 0.03, 105, 0.05, 0.05);
            rclcpp::sleep_for(std::chrono::seconds(4));

            // <group_state name = "ready" group = "${group_name}">
            //     <joint name = "${arm_id}_joint1" value = "0" />
            //     <joint name = "${arm_id}_joint2" value = "${-pi/4}" />
            //     <joint name = "${arm_id}_joint3" value = "0" />
            //     <joint name = "${arm_id}_joint4" value = "${-3*pi/4}" />
            //     <joint name = "${arm_id}_joint5" value = "0" />
            //     <joint name = "${arm_id}_joint6" value = "${pi/2}" />
            //     <joint name = "${arm_id}_joint7" value = "${pi/4}" />
            //     </ group_state>
            // Move back to the ready position to make sure transformation to cup is correct
            // moveArmToPose(move_group_interface_arm, ready_pose, logger);

            std::vector<double> target_joint_positions = {0.0, -0.785, 0, -2.356, 0, 1.571, 0.785};
            moveArmToJointPositions(move_group_interface_arm, target_joint_positions, logger);

            rclcpp::sleep_for(std::chrono::seconds(5));

            // Move to the cup to deposit the ball
            // - -0.13451694061274677
            // - 0.07687164586573317
            // - -0.3061883953579685
            // - -2.4008782395145136
            // - -0.03365871842371093
            // - 2.4790453498098586
            // - 0.6065877616786295
            // moveArmToPose(move_group_interface_arm, deposit_pose, logger);
            std::vector<double> deposit_joint_positions = {-0.13451, 0.07687, -0.30618, -2.40087, -0.03365, 2.47905, 0.60658};
            moveArmToJointPositions(move_group_interface_arm, deposit_joint_positions, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));

            // Release the ball
            gripper_node->sendMoveCommand(0.08, 0.03);
            rclcpp::sleep_for(std::chrono::seconds(4));

            // Move back to the ready position to get ready for the next ball
            // moveArmToPose(move_group_interface_arm, ready_pose, logger);
            moveArmToJointPositions(move_group_interface_arm, target_joint_positions, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));
            // }
        }

        rclcpp::sleep_for(std::chrono::milliseconds(50)); // Adjust print rate
    }

    // rclcpp::spin(arm_node);
    // rclcpp::spin(gripper_node);

    // Shutdown ROS
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}