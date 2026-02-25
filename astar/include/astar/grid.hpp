#pragma once

#include <vector>
#include <cmath>

namespace astar
{

enum class CellState {
	FREE,
	OBSTACLE,
	OPEN,
	CLOSED,
	PATH,
	START,
	GOAL
};

struct Cell{
	int x;
	int y;
	CellState state = CellState::FREE;

	//values of A*
	float g = 0.0f;
	float h = 0.0f;
	float f = 0.0f; // g + h
	
	int parent_x = -1;
	int parent_y = -1;

        bool operator>(const Cell & other) const { return f > other.f;} 
};

class Grid
{
	public:
	int width;
	int height;
	std::vector<std::vector<Cell>> cells;

	Grid(int width, int height) : width(width), height(height)
	{
		cells.resize( height , std::vector<Cell>(width));
		for(int y = 0; y < height ; y++){
		  for(int x = 0; x < width ; x++){
			  cells[y][x].x = x;
			  cells[y][x].y = y;
		  }
		}
	}

	bool in_bound(int x, int y) const
	{  return x >= 0 && x < width && y >= 0 && y < height;}

	void set_obstacle(int x, int y)
	{
	  if(in_bound(x,y))
	  { cells[y][x].state = CellState::OBSTACLE;}
	}

	void set_obstacle_rectangle(int x0, int y0, int x1, int y1)
	{
	  for( int y = y0 ; y <= y1 ; y++ )
	  {  for(int x = x0 ; x <= x1; x++)
	     {  set_obstacle(x,y); }
	  }
	} 
	
	bool is_walkable(int x, int y) const
	{
	  return in_bound(x, y) && cells[y][x].state != CellState::OBSTACLE;
	}

	Cell & get(int x, int y)
	{  return cells[y][x]; }

	const Cell & get(int x, int y) const
	{  return cells[y][x]; }

	static float heuristic(int x1, int y1, int x2, int y2)
	{
	   return std::abs(x2-x1)+std::abs(y2-y1);
	}
}; 
}

