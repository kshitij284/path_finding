#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "astar/grid.hpp"
#include "astar/astar.hpp"
#include <vector>
#include <utility>

using namespace std::chrono_literals;

class AstarNode : public rclcpp::Node
{
  public:
  AstarNode():Node("astar_node")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/grid_marker",10);
    timer_ = this->create_wall_timer(500ms,[this](){this->publish_grid();});

    grid_ = std::make_unique<astar::Grid>(20,20);
    grid_->set_obstacle_rectangle(4,4,4,11);
    grid_->set_obstacle_rectangle(10,7,15,7);
    grid_->set_obstacle_rectangle(14,1,14,6);

    grid_->get(1,1).state = astar::CellState::START;
    grid_->get(17,19).state = astar::CellState::GOAL;
    
    std::vector<std::pair<int,int>> path = run_astar(*grid_, 1, 1, 17, 19);
    for (auto pair : path)
    {
	 grid_->get(pair.first,pair.second).state = astar::CellState::PATH;
    }	 
  }

  private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<astar::Grid> grid_;
  void publish_grid()
  {
    visualization_msgs::msg::MarkerArray msg;
    int id = 0;
    for(int y = 0; y < grid_->height; y++)
    { for(int x = 0; x < grid_->width; x++)
        { 
	  visualization_msgs::msg::Marker marker;
	  marker.header.frame_id = "map";
	  marker.header.stamp = this->now();
	  marker.ns = "grid";
	  marker.id = id++;
	  marker.type = visualization_msgs::msg::Marker::CUBE;
	  marker.action = visualization_msgs::msg::Marker::ADD;
	  
	  marker.pose.position.x = static_cast<double>(x);
	  marker.pose.position.y = static_cast<double>(y);
	  marker.pose.position.z = 0.0;
	  marker.pose.orientation.w = 1.0;

	  marker.scale.x = 0.9;
	  marker.scale.y = 0.9;
	  marker.scale.z = 0.1;
	  
	  // Color by cell state
          marker.color.a = 1.0f;
	  const auto & cell = grid_->get(x, y);
          
	  switch (cell.state) {
              case astar::CellState::FREE:
                  marker.color.r = 0.3f; marker.color.g = 0.3f; marker.color.b = 0.3f;
                  break;
              case astar::CellState::OBSTACLE:
                  marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f;
                  break;
              case astar::CellState::OPEN:
                  marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
                  break;
              case astar::CellState::CLOSED:
                  marker.color.r = 0.0f; marker.color.g = 0.0f; marker.color.b = 1.0f;
                  break;
              case astar::CellState::PATH:
                  marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f;
                  break;
              case astar::CellState::START:
                  marker.color.r = 1.0f; marker.color.g = 1.0f; marker.color.b = 1.0f;
                  break;
              case astar::CellState::GOAL:
                  marker.color.r = 1.0f; marker.color.g = 0.5f; marker.color.b = 0.0f;
                  break;
          }
	  msg.markers.push_back(marker);
	}
    }
    publisher_->publish(msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstarNode>());
  rclcpp::shutdown();
  return 0;
}
