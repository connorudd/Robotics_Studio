#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cmath>
#include <vector>

class CylinderDetector : public rclcpp::Node {
public:
    CylinderDetector() : Node("cylinder_detector") {
        // Subscribe to the laser scan data
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/visualization_marker", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const float CLUSTER_THRESHOLD = 0.06;  // 6 cm threshold for clustering
        const int MIN_CLUSTER_SIZE = 10;  // Minimum number of points for a valid object
        const float CYLINDER_DIAMETER = 0.30;  // 30 cm diameter

        std::vector<std::vector<int>> clusters;
        std::vector<int> current_cluster;

        // Group adjacent points into clusters
        for (int i = 1; i < msg->ranges.size(); ++i) {
            if (isValidRange(msg->ranges[i]) && 
                std::fabs(msg->ranges[i] - msg->ranges[i - 1]) < CLUSTER_THRESHOLD) {
                current_cluster.push_back(i);
            } else {
                if (current_cluster.size() >= MIN_CLUSTER_SIZE) {
                    clusters.push_back(current_cluster);
                }
                current_cluster.clear();
            }
        }
        // Check the last cluster
        if (current_cluster.size() >= MIN_CLUSTER_SIZE) {
            clusters.push_back(current_cluster);
        }

        // Process clusters to find potential cylinders
        for (const auto& cluster : clusters) {
            // Calculate average distance and angle of the cluster
            float avg_angle = 0.0, avg_distance = 0.0;
            for (int idx : cluster) {
                float angle = msg->angle_min + idx * msg->angle_increment;
                avg_angle += angle;
                avg_distance += msg->ranges[idx];
            }
            avg_angle /= cluster.size();
            avg_distance /= cluster.size();

            // Convert polar coordinates to Cartesian (x, y)
            float x = avg_distance * std::cos(avg_angle);
            float y = avg_distance * std::sin(avg_angle);

            // Publish marker if the detected object matches the expected diameter
            if (clusterMatchesCylinder(cluster, msg)) {
                publishMarker(x, y);
            }
        }
    }

    bool isValidRange(float range) {
        return !std::isinf(range) && !std::isnan(range) && range > 0.1 && range < 10.0;
    }

    bool clusterMatchesCylinder(const std::vector<int>& cluster, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Estimate the size of the object based on the angle span of the cluster
        float angle_span = (cluster.back() - cluster.front()) * msg->angle_increment;
        float estimated_diameter = 2.0 * msg->ranges[cluster[0]] * std::sin(angle_span / 2.0);

        // Check if the estimated diameter matches the expected cylinder diameter (within tolerance)
        return std::fabs(estimated_diameter - 0.30) < 0.05;  // 5 cm tolerance
    }

    void publishMarker(float x, float y) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/scan";  // Ensure frame ID matches the laser scan
        marker.header.stamp = this->now();
        marker.ns = "cylinder";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the marker position and scale
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.5;  // Height above ground
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.30;  // Diameter
        marker.scale.y = 0.30;  // Diameter
        marker.scale.z = 1.0;   // Height

        marker.color.a = 1.0;  // Fully opaque
        marker.color.r = 1.0;  // Red color
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        RCLCPP_INFO(this->get_logger(), "Cylinder detected at: (%.2f, %.2f)", x, y);
        marker_pub_->publish(marker);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderDetector>());
    rclcpp::shutdown();
    return 0;
}

