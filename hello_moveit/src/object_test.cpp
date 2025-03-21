#include <rclcpp/rclcpp.hpp>
#include <zed_msgs/msg/objects_stamped.hpp>
#include <vector>
#include <string>
#include <array>
#include <thread>

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

        RCLCPP_INFO(this->get_logger(), "Stored %ld objects", detected_objects_.size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ObjectDetectionSubscriber>();

    // Spin node in a separate thread to keep receiving messages
    std::thread spin_thread([&]()
                            { rclcpp::spin(node); });

    while (rclcpp::ok())
    {
        auto objects = node->getDetectedObjects();

        for (const auto &obj : objects)
        {
            RCLCPP_INFO(node->get_logger(),
                        "Object: %s (ID: %d) - Position: x=%.4f, y=%.4f, z=%.4f",
                        obj.label.c_str(), obj.id, obj.position[0], obj.position[1], obj.position[2]);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(500)); // Adjust print rate
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}