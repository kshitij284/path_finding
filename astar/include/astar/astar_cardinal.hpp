#pragma once 

#include "astar/grid.hpp"
#include <set>
#include <queue>
#include <utility>
#include <vector>
#include <algorithm>

std::vector<std::pair<int,int>> run_astar(astar::Grid& grid, int start_x, int start_y, int goal_x, int goal_y)
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
    astar::Cell current = open_set.top();
    open_set.pop();
    
    if(current.x == goal_x && current.y == goal_y)
    {
      std::vector<std::pair<int,int>> path = {{current.x,current.y}};
      int parent_x_now = current.parent_x;
      int parent_y_now = current.parent_y;
      while(parent_x_now != -1 && parent_y_now != -1)
      {
        const astar::Cell path_cell = grid.get(parent_x_now, parent_y_now);
        std::pair<int,int> parent_now = {parent_x_now, parent_y_now};
	    path.push_back(parent_now);
	    parent_x_now = path_cell.parent_x;
	    parent_y_now = path_cell.parent_y;
      }
    std::reverse(path.begin(), path.end());
    return path;
    }
    
    if(closed_set.count({current.x, current.y})) {continue;}
    else
    {
      closed_set.insert({ current.x, current.y });
      //four neighbors
      
      std::vector<std::vector<int>> sides_updates = {{0,-1},{0,+1},{-1,0},{+1,0}};
      
      for (auto side : sides_updates)
      {   
        int neighbor_x = current.x+side[0];
        int neighbor_y = current.y+side[1];

        if(grid.in_bound(neighbor_x, neighbor_y))
	{  
	  astar::Cell & neighbor_cell = grid.get(neighbor_x, neighbor_y);
	  if(!closed_set.count({neighbor_x, neighbor_y}) && grid.is_walkable(neighbor_x, neighbor_y))
          {
	    neighbor_cell.g = current.g + 1;
	    neighbor_cell.h = astar::Grid::heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
	    neighbor_cell.f = neighbor_cell.g + neighbor_cell.h;
            neighbor_cell.parent_x = current.x;
	    neighbor_cell.parent_y = current.y;
            open_set.push(neighbor_cell);
	  }
	}
      } 
    }
  }
  return {};
}

std::vector<std::pair<int,int>> step_astar(astar::Grid& grid,std::priority_queue<astar::Cell, std::vector<astar::Cell>, std::greater<astar::Cell>>& open_set, std::set<std::pair<int,int>>& closed_set, int goal_x, int goal_y)
{
   
    astar::Cell current = open_set.top();
    open_set.pop();
    
    if(current.x == goal_x && current.y == goal_y)
    {
      std::vector<std::pair<int,int>> path = {{current.x,current.y}};
      int parent_x_now = current.parent_x;
      int parent_y_now = current.parent_y;
      while(parent_x_now != -1 && parent_y_now != -1)
      {
        const astar::Cell path_cell = grid.get(parent_x_now, parent_y_now);
        std::pair<int,int> parent_now = {parent_x_now, parent_y_now};
	    path.push_back(parent_now);
	    parent_x_now = path_cell.parent_x;
	    parent_y_now = path_cell.parent_y;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    
    if(closed_set.count({current.x, current.y})) {return {};}
    else
    {
      closed_set.insert({ current.x, current.y });
      grid.get(current.x, current.y).state = astar::CellState::CLOSED;

      //four neighbors
      
      std::vector<std::vector<int>> sides_updates = {{0,-1},{0,+1},{-1,0},{+1,0}};
      
      for (auto side : sides_updates)
      {   
        int neighbor_x = current.x+side[0];
        int neighbor_y = current.y+side[1];

        if(grid.in_bound(neighbor_x, neighbor_y))
	{  
	  astar::Cell & neighbor_cell = grid.get(neighbor_x, neighbor_y);
	  if(!closed_set.count({neighbor_x, neighbor_y}) && grid.is_walkable(neighbor_x, neighbor_y))
          {
	    neighbor_cell.g = current.g + 1;
	    neighbor_cell.h = astar::Grid::heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
	    neighbor_cell.f = neighbor_cell.g + neighbor_cell.h;
            neighbor_cell.parent_x = current.x;
	    neighbor_cell.parent_y = current.y;
	    neighbor_cell.state = astar::CellState::OPEN;
            open_set.push(neighbor_cell);
	  }
	}
      } 
    }
  return {};
}
