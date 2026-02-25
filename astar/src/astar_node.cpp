#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "astar/grid.hpp"

using namespace std::chrono_literals;

class AstarNode : public rclcpp::Node
{
  public:
  AstarNode():Node("astar_node")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/grid_marker",10);
    timer_ = this->create_wall_timer(500ms,[this](){this->timer_callback();});
  }

  private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<astar::Grid> grid_;
  void timer_callback(){;};

};
