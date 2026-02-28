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
    
    grid_->get(start_x_,start_y_).state = astar::CellState::START;
    grid_->get(goal_x_,goal_y_).state = astar::CellState::GOAL;

    astar::Cell start_cell;
    start_cell.x = start_x_;
    start_cell.y = start_y_;
    start_cell.state = astar::CellState::START;
    start_cell.g = 0.0f;
    start_cell.h = astar::Grid::heuristic(start_x_, start_y_, goal_x_, goal_y_);
    start_cell.f = start_cell.g + start_cell.h;
    open_set_.push(start_cell);
    
    /*std::vector<std::pair<int,int>> path = run_astar(*grid_, 1, 1, 17, 19);
    for (auto pair : path)
    {
	 grid_->get(pair.first,pair.second).state = astar::CellState::PATH;
    }*/     
  }

  private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<astar::Grid> grid_;
  std::priority_queue<astar::Cell, std::vector<astar::Cell>, std::greater<astar::Cell>> open_set_;
  std::set<std::pair<int,int>> closed_set_;
  bool search_complete = false;
  int start_x_ = 1;
  int start_y_ = 1;
  int goal_x_ = 17;
  int goal_y_ = 19;

  void publish_grid()
  { 
    if(!search_complete)
    {  std::vector<std::pair<int,int>> path = step_astar(*grid_, open_set_, closed_set_, goal_x_, goal_y_);
       if(!path.empty())
       {  for(auto p : path)
	  { grid_ ->get(p.first, p.second).state = astar::CellState::PATH ; }
	  search_complete = true;
       }
    }

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
