#pragma once
#include <vector>
#include <set>
#include <queue>
#include "grid.hpp"
#include <algorithm>

std::vector<std::pair<int,int>> run_astar(astar::Grid& grid, int start_x, int start_y, int goal_x, int goal_y,bool allow_diagonal = false)
{
  astar::Cell start_cell;
  start_cell.x = start_x;
  start_cell.y = start_y;
  start_cell.state = astar::CellState::START;
  start_cell.g = 0.0f;
  start_cell.h = astar::Grid::heuristic(start_x, start_y, goal_x, goal_y);
  start_cell.f = start_cell.g + start_cell.h;

  std::priority_queue<astar::Cell, std::vector<astar::Cell>, std::greater<astar::Cell>> open_set;
  std::set<std::pair<int,int>> closed_set;

  open_set.push(start_cell);

  while(!open_set.empty())
  {
    astar::Cell current_cell = open_set.top();
    open_set.pop();

    if(current_cell.x == goal_x && current_cell.y == goal_y)
    {
      int parent_x_now = current_cell.parent_x;
      int parent_y_now = current_cell.parent_y;
      std::vector<std::pair<int,int>> path = {{current_cell.x, current_cell.y}};
      while(parent_x_now != -1 && parent_y_now != -1)
      {
        path.push_back({parent_x_now, parent_y_now});
        const astar::Cell path_cell = grid.get(parent_x_now, parent_y_now);
        parent_x_now = path_cell.parent_x;
        parent_y_now = path_cell.parent_y;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    if(closed_set.count({current_cell.x, current_cell.y})) { continue; }
    else
    {
      closed_set.insert({current_cell.x, current_cell.y});

      std::vector<std::pair<int,int>> side_update = {
        { 1, 0}, {-1, 0}, { 0, 1}, { 0,-1},   // cardinal
      };
      if(allow_diagonal) {
	    side_update.push_back({1,1});
	    side_update.push_back({1,-1});
	    side_update.push_back({-1,1});
	    side_update.push_back({-1,-1});
	}

      for(auto neighbor : side_update)
      {
        int neighbor_cell_x = current_cell.x + neighbor.first;
        int neighbor_cell_y = current_cell.y + neighbor.second;

        if(grid.in_bound(neighbor_cell_x, neighbor_cell_y))
        {
          astar::Cell& neighbor_cell = grid.get(neighbor_cell_x, neighbor_cell_y);

          if(grid.is_walkable(neighbor_cell_x, neighbor_cell_y) && !closed_set.count({neighbor_cell_x, neighbor_cell_y}))
          {
            // check if diagonal move
            bool is_diagonal = (neighbor.first != 0 && neighbor.second != 0);

            // block corner cutting through walls
            if(is_diagonal)
            {
              bool x_blocked = !grid.is_walkable(current_cell.x + neighbor.first, current_cell.y);
              bool y_blocked = !grid.is_walkable(current_cell.x, current_cell.y + neighbor.second);
              if(x_blocked || y_blocked) { continue; }
            }

            float move_cost = is_diagonal ? 1.414f : 1.0f;
            neighbor_cell.g = current_cell.g + move_cost;
            neighbor_cell.h = astar::Grid::heuristic(neighbor_cell_x, neighbor_cell_y, goal_x, goal_y);
            neighbor_cell.f = neighbor_cell.g + neighbor_cell.h;
            neighbor_cell.parent_x = current_cell.x;
            neighbor_cell.parent_y = current_cell.y;
            open_set.push(neighbor_cell);
          }
        }
      }
    }
  }
  return {};
}

// runs one step of A* — call repeatedly from your ROS2 timer callback
// returns empty vector {} if path not found yet, returns path when goal is reached
std::vector<std::pair<int,int>> step_astar(
  astar::Grid& grid,
  std::priority_queue<astar::Cell, std::vector<astar::Cell>, std::greater<astar::Cell>>& open_set,
  std::set<std::pair<int,int>>& closed_set,
  int goal_x, int goal_y, bool allow_diagonal = false)
{
  if(open_set.empty()) { return {}; }

  astar::Cell current = open_set.top();
  open_set.pop();

  // goal reached — trace path back via parent pointers
  if(current.x == goal_x && current.y == goal_y)
  {
    std::vector<std::pair<int,int>> path = {{current.x, current.y}};
    int parent_x_now = current.parent_x;
    int parent_y_now = current.parent_y;
    while(parent_x_now != -1 && parent_y_now != -1)
    {
      path.push_back({parent_x_now, parent_y_now});
      const astar::Cell path_cell = grid.get(parent_x_now, parent_y_now);
      parent_x_now = path_cell.parent_x;
      parent_y_now = path_cell.parent_y;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  // already processed this cell
  if(closed_set.count({current.x, current.y})) { return {}; }

  closed_set.insert({current.x, current.y});
  grid.get(current.x, current.y).state = astar::CellState::CLOSED;

  std::vector<std::pair<int,int>> side_update = {
    { 1, 0}, {-1, 0}, { 0, 1}, { 0,-1},   // cardinal
  };
  if(allow_diagonal) {
	    side_update.push_back({1,1});
	    side_update.push_back({1,-1});
	    side_update.push_back({-1,1});
	    side_update.push_back({-1,-1});
	}

  for(auto side : side_update)
  {
    int neighbor_x = current.x + side.first;
    int neighbor_y = current.y + side.second;

    if(grid.in_bound(neighbor_x, neighbor_y))
    {
      astar::Cell& neighbor_cell = grid.get(neighbor_x, neighbor_y);

      if(grid.is_walkable(neighbor_x, neighbor_y) && !closed_set.count({neighbor_x, neighbor_y}))
      {
        bool is_diagonal = (side.first != 0 && side.second != 0);

        // block corner cutting through walls
        if(is_diagonal)
        {
          bool x_blocked = !grid.is_walkable(current.x + side.first, current.y);
          bool y_blocked = !grid.is_walkable(current.x, current.y + side.second);
          if(x_blocked || y_blocked) { continue; }
        }

        float move_cost = is_diagonal ? 1.414f : 1.0f;
        neighbor_cell.g = current.g + move_cost;
        neighbor_cell.h = astar::Grid::heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
        neighbor_cell.f = neighbor_cell.g + neighbor_cell.h;
        neighbor_cell.parent_x = current.x;
        neighbor_cell.parent_y = current.y;
        neighbor_cell.state = astar::CellState::OPEN;
        open_set.push(neighbor_cell);
      }
    }
  }
  return {};
}

