#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <proj.h>
#include <Eigen/Dense>
#include <csv.h> // CSV読み込み用のライブラリを仮定
#include <vector>

class GPSPathGenerator : public rclcpp::Node {
public:
    GPSPathGenerator() : Node("gps_path_generator") {
        proj_context_create();
        proj_create(PJ_DEFAULT_CTX, "EPSG:4326"); // WGS84
        proj_create(PJ_DEFAULT_CTX, "EPSG:32633"); // UTM

        auto file_path = "/path/to/gazebo_compress.csv";
        auto df = read_csv(file_path); // CSV読み込みの仮定実装

        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "map";

        double base_x = 0.0, base_y = 0.0;
        bool first = true;

        for (const auto& data : df) {
            auto [x, y] = convert_gps_to_utm(data.latitude, data.longitude);

            if (first) {
                base_x = x;
                base_y = y;
                first = false;
            }

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";
            pose.pose.position.x = x - base_x;
            pose.pose.position.y = y - base_y;
            path.poses.push_back(pose);
        }

        this->get_logger().info("Data loaded and path published");
        auto publisher = this->create_publisher<nav_msgs::msg::Path>("gnss_path", 10);
        publisher->publish(path);
    }

private:
    std::pair<double, double> convert_gps_to_utm(double lat, double lon) {
        // 座標変換ロジックを実装
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSPathGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

