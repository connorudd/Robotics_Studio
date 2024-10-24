// Include the ROS2 C++ client library
// Include the LaserScan message type
#include <rclcpp/rclcpp.hpp>                         
#include <sensor_msgs/msg/laser_scan.hpp>            

// Define a class that inherits from rclcpp::Node to create a ROS2 node
class LaserScanProcessor : public rclcpp::Node
{
public:
    // Constructor for the LaserScanProcessor node
    LaserScanProcessor()
        : Node("laser_scan_processor") // Initialize the node with the name "laser_scan_processor"
    {
        // Create a subscription to the "/scan" topic with a queue size of 10
        // The callback function 'scanCallback' will be called whenever a new message is received
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the first subset of the laser scan on the "/scan_subset_1" topic
        scan_pub_1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset_1", 10);
        
        // Create a publisher for the second subset of the laser scan on the "/scan_subset_2" topic
        scan_pub_2_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset_2", 10);
    }

private:
    // Callback function that processes incoming LaserScan messages
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Log the minimum angle of the scan in radians and degrees
        RCLCPP_INFO(this->get_logger(), "angle_min: %f radians (%f degrees)", scan->angle_min, scan->angle_min * 180.0 / M_PI);
        
        // Log the maximum angle of the scan in radians and degrees
        RCLCPP_INFO(this->get_logger(), "angle_max: %f radians (%f degrees)", scan->angle_max, scan->angle_max * 180.0 / M_PI);
        
        // Log the angle increment between measurements in radians and degrees
        RCLCPP_INFO(this->get_logger(), "angle_increment: %f radians (%f degrees)", scan->angle_increment, scan->angle_increment * 180.0 / M_PI);

        // Define the desired angle ranges for the first subset in degrees
        double angle_min_deg_1 = 0.0;
        double angle_max_deg_1 = 30.0;
        
        // Define the desired angle ranges for the second subset in degrees
        double angle_min_deg_2 = 330.0;
        double angle_max_deg_2 = 360.0;

        // Convert the first subset angle ranges from degrees to radians
        double angle_min_rad_1 = angle_min_deg_1 * M_PI / 180.0;
        double angle_max_rad_1 = angle_max_deg_1 * M_PI / 180.0;
        
        // Convert the second subset angle ranges from degrees to radians
        double angle_min_rad_2 = angle_min_deg_2 * M_PI / 180.0;
        double angle_max_rad_2 = angle_max_deg_2 * M_PI / 180.0;

        // Calculate the start and end indices for the first subset based on the angle ranges
        int start_index_1 = static_cast<int>((angle_min_rad_1 - scan->angle_min) / scan->angle_increment);
        int end_index_1 = static_cast<int>((angle_max_rad_1 - scan->angle_min) / scan->angle_increment);
        
        // Calculate the start and end indices for the second subset based on the angle ranges
        int start_index_2 = static_cast<int>((angle_min_rad_2 - scan->angle_min) / scan->angle_increment);
        int end_index_2 = static_cast<int>((angle_max_rad_2 - scan->angle_min) / scan->angle_increment);

        // Create a new LaserScan message for the first subset by copying the original scan
        auto subset_scan_1 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        
        // Extract the range data corresponding to the first subset
        subset_scan_1->ranges = std::vector<float>(scan->ranges.begin() + start_index_1, scan->ranges.begin() + end_index_1 + 1);
        
        // Update the angle_min and angle_max for the first subset
        subset_scan_1->angle_min = scan->angle_min + start_index_1 * scan->angle_increment;
        subset_scan_1->angle_max = scan->angle_min + end_index_1 * scan->angle_increment;

        // Create a new LaserScan message for the second subset by copying the original scan
        auto subset_scan_2 = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        
        // Extract the range data corresponding to the second subset
        subset_scan_2->ranges = std::vector<float>(scan->ranges.begin() + start_index_2, scan->ranges.begin() + end_index_2 + 1);
        
        // Update the angle_min and angle_max for the second subset
        subset_scan_2->angle_min = scan->angle_min + start_index_2 * scan->angle_increment;
        subset_scan_2->angle_max = scan->angle_min + end_index_2 * scan->angle_increment;

        // Publish the first subset LaserScan message
        scan_pub_1_->publish(*subset_scan_1);
        
        // Publish the second subset LaserScan message
        scan_pub_2_->publish(*subset_scan_2);
    }

    // Subscriber to the LaserScan messages
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    // Publisher for the first subset of LaserScan messages
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_1_;
    
    // Publisher for the second subset of LaserScan messages
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_2_;
};

// The main function initializes the ROS2 system and starts the LaserScanProcessor node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    
    return 0;
}
