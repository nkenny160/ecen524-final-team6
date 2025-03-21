//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
// PROTOTYPES FOR GRIPPER CONTROL
//------------------------------------------------------------------------------

// This class constructs a node to communicate with the gripper on the Franka Emika Panda. It also provides helper functions to send homing, grasp, and move commands to the robot.
class GripperControlNode : public rclcpp::Node
{
public:
    using Grasp = franka_msgs::action::Grasp;
    using Homing = franka_msgs::action::Homing;
    using Move = franka_msgs::action::Move;
    using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;
    using GoalHandleHoming = rclcpp_action::ClientGoalHandle<Homing>;
    using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

    // Initial call constructs the node with the three desired clients for homing, grasping, and moving
    GripperControlNode() : Node("gripper_client")
    {
        grasp_client_ = rclcpp_action::create_client<Grasp>(this, "/panda_gripper/grasp");
        homing_client_ = rclcpp_action::create_client<Homing>(this, "/panda_gripper/homing");
        move_client_ = rclcpp_action::create_client<Move>(this, "/panda_gripper/move");
    }

    // Helper function to send a grasp command to the robot
    void sendGraspCommand(double width, double speed, double force, double epsilon_inner, double epsilon_outer)
    {
        // Wait for the action server to become available
        if (!grasp_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Grasp action server not available!");
            return;
        }

        // Construct the message
        auto goal_msg = Grasp::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;
        goal_msg.force = force;
        goal_msg.epsilon.inner = epsilon_inner;
        goal_msg.epsilon.outer = epsilon_outer;

        // Send the message and report status
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

    // Helper function to send a homing command to the robot
    void sendHomingCommand()
    {
        // Wait for the action server to become available
        if (!homing_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Homing action server not available!");
            return;
        }

        // Construct the message
        auto goal_msg = Homing::Goal();

        // Send the message and report status
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

    // Helper function to send a move command to the robot
    void sendMoveCommand(double width, double speed)
    {
        // Wait for the action server to become available
        if (!move_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Move action server not available!");
            return;
        }

        // Construct the message
        auto goal_msg = Move::Goal();
        goal_msg.width = width;
        goal_msg.speed = speed;

        // Send the message and report status
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

//------------------------------------------------------------------------------
// PROTOTYPES FOR ARM CONTROL
//------------------------------------------------------------------------------

// Helper function that takes as input a target move_group and a target pose (orientation and translation), and plans and executes a motion for the target move_group to reach the target pose
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

// Helper function that takes as input a target move_group and a target pose (joint positions for all joints in move_group), and plans and executes a motion for the target move_group to reach the target pose
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

//------------------------------------------------------------------------------
// PROTOTYPES FOR OBJECT DETECTION
//------------------------------------------------------------------------------

// Define a struct to hold object data (ID, label, and position)
struct ObjectData
{
    int id;                        // Object ID
    std::string label;             // Object label
    std::array<float, 3> position; // Object position (x, y, z)
};

// This class constructs a ROS node that subscribes to the ZED object detection "objects" topic and recieves the desired information.
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

//------------------------------------------------------------------------------
// MAIN
//------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create arm, gripper, and object detection nodes
    auto const arm_node = std::make_shared<rclcpp::Node>(
        "auto_pick_place_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto gripper_node = std::make_shared<GripperControlNode>();

    auto object_node = std::make_shared<ObjectDetectionSubscriber>();

    // Create the MoveIt MoveGroup Interface for Arm
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface_arm = MoveGroupInterface(arm_node, "panda_arm");
    move_group_interface_arm.setPlanningTime(10);

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create an a collision object in the robot base frame
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface_arm.getPlanningFrame();
    collision_object.id = "box1";

    // Define the shape of the collision object. The purpose of this collision object is to prevent the end effector from making contact with the table during motion plan execution. We need this because the end effector attachments we are using are slightly longer than the native Franka Emika attachments, so the robot might plan a path that causes our custom end effector to make contact with the table.
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    // X and Y are arbitrarily set
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    // This z dimension ensures that the end effector will not contact the table
    primitive.dimensions[primitive.BOX_Z] = 0.17;

    // Define the pose of the object in the base frame
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    // Add the object to the planning scene. Now all future motion planning will avoid contact with this object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Home the gripper (calibration)
    gripper_node->sendHomingCommand();
    rclcpp::sleep_for(std::chrono::seconds(10));

    // Create a multi threaded executor so that our three ROS nodes can run concurrently and not cause delays
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(arm_node);
    executor.add_node(gripper_node);
    executor.add_node(object_node);

    // Run multi threaded executor in a separate thread
    std::thread executor_thread([&executor]()
                                { executor.spin(); });

    // Define the Homogeneous Transformation Matrix of the ZED Camera Frame wrt the robot base frame. This is how we convert our measured position from ZED into a position that can be used for planning a path for the robot 
    Eigen::Matrix4f T1_0;
    T1_0 << 0.0, 1.0, 0.0, 0.2016,
        -0.9511, 0.0, -0.3090, 1.0604,
        -0.3090, 0.0, 0.9511, 0.3429,
        0.0, 0.0, 0.0, 1.0;

    // This loop will continue as long as ROS is running or until the user terminates the code. This means that after a pick and place is executed, if the user places another ball in the workspace, the robot will find the new position and perform pick and place again
    while (rclcpp::ok())
    {
        // Get the most recent list of all detected objects from ZED
        auto objects = object_node->getDetectedObjects();

        // Loop trough all objects
        for (const auto &obj : objects)
        {
            // Display the infor for debugging
            RCLCPP_INFO(arm_node->get_logger(),
                        "Object: %s (ID: %d) - Position: x=%.2f, y=%.2f, z=%.2f",
                        obj.label.c_str(), obj.id, obj.position[0], obj.position[1], obj.position[2]);

            // If object label does not match the desired label, skip the following code and continue with the next detected object
            if (obj.label != "SPORT")
                continue;

            // Construct the position vector of the object in the ZED camera frame. The 1.0 is added as the fourth element so that we can multiply this vector with the transformation matrix
            Eigen::Vector4f obj_pos_cam_frame(static_cast<float>(obj.position[0]),
                                              static_cast<float>(obj.position[1]),
                                              static_cast<float>(obj.position[2]),
                                              1.0);

            // Calculate the position vector of the object in the robot base frame
            Eigen::Vector4f obj_pos_base_frame = T1_0 * obj_pos_cam_frame;

            float new_x = obj_pos_base_frame(0);
            float new_y = obj_pos_base_frame(1);

            // Set a target Pose (pickup the ball)
            geometry_msgs::msg::Pose new_target_pose;
            // Since the object (ball) is symmetrical, we can use a constant desired orientation
            new_target_pose.orientation.x = 0.74868;
            new_target_pose.orientation.y = 0.66245;
            new_target_pose.orientation.z = 0.021153;
            new_target_pose.orientation.w = 0.013489;
            // Since the object (ball) is on a flat surface, we can use a constant desired z position
            new_target_pose.position.x = new_x; // from vision
            new_target_pose.position.y = new_y; // from vision;
            new_target_pose.position.z = 0.17181293666362762;

            std::cout << "Resulting vector:\n"
                      << obj_pos_base_frame << std::endl;

            std::cout << "Pick and Place task started... " << std::endl;

            // The delays throughout this process ensure that the robot has time to plan and execute each path before the next step begins, ensuring no conflicts

            // Move to the ball
            moveArmToPose(move_group_interface_arm, new_target_pose, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));

            // Grab the ball
            gripper_node->sendGraspCommand(0.025, 0.03, 105, 0.05, 0.05);
            rclcpp::sleep_for(std::chrono::seconds(4));

            // Move to home position
            std::vector<double> home_joint_positions = {0.0, -0.785, 0, -2.356, 0, 1.571, 0.785};
            moveArmToJointPositions(move_group_interface_arm, home_joint_positions, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));

            // Move to deposit position
            std::vector<double> deposit_joint_positions = {-0.13451, 0.07687, -0.30618, -2.40087, -0.03365, 2.47905, 0.60658};
            moveArmToJointPositions(move_group_interface_arm, deposit_joint_positions, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));

            // Release the ball
            gripper_node->sendMoveCommand(0.08, 0.03);
            rclcpp::sleep_for(std::chrono::seconds(4));

            // Move back to the ready position to get ready for the next ball
            moveArmToJointPositions(move_group_interface_arm, home_joint_positions, logger);
            rclcpp::sleep_for(std::chrono::seconds(5));
        }

        rclcpp::sleep_for(std::chrono::milliseconds(50)); // Adjust rate
    }

    // Shutdown ROS
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}