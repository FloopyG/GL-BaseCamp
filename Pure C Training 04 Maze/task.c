#include <stdio.h>
#include <stdlib.h>
#include "maze.h"

typedef enum direction {
  TOP,
  LEFT,
  RIGHT,
  BOTTOM
} direction;


// For more cool colorful console output
void red ()
{
  printf("\033[0;31m");
}

void green ()
{
  printf("\033[0;32m");
}

void yellow ()
{
  printf("\033[0;33m");
}

/*

  Function is able to find path from point A(x, y) to point B(x, y) in maze. 
  Maze must be stored in form of array and consist of 1 (walls) and 0 (path).
  Paths can`t be wider that 1 "square", or the algrotihm won`t work. If the work were succesful it will
  return 1 and change the path "squares" values in array for 2 (other methods of storing path data can be used). 
  If there is no path it will return 0.
  Algorithm will also break if there is now walls around the maze(can be fixed).
  Algorithm is only looking for any available path, not the shortest one.

*/
int findPath (int maze[HEIGHT][WIDTH], int current_X, int current_Y, int exit_X, int exit_Y, direction dir) 
{
  // Checking if current point is our destination
  if (current_X == exit_X && current_Y == exit_Y) 
  {
    return 1;
  }

  // Changing current position to 3, to mark passed ways and avoid infinite loops
  maze[current_Y][current_X] = 3;

  // To see full algorithm path (x, y format)
  // printf("%d\n x: %d\n y: %d\n", maze[current_Y][current_X], current_X + 1, current_Y + 1);


  // Depending from where it came, choose which neigbours must be scanned. Never scannes were it came from.
  // Basically moving our position in all possible ways to find a destination. 
  // If all possible available coordinates were checked returns 0.
  // When we arrive at our destination we change the path values to 2, using backtracking.
  if (maze[current_Y][current_X - 1] == 0 && dir != LEFT) {
    if (findPath(maze, current_X - 1, current_Y, exit_X, exit_Y, RIGHT) == 1)
    {
      maze[current_Y][current_X - 1] = 2;
      return 1;
    }
  }
  if (maze[current_Y][current_X + 1] == 0 && dir != RIGHT) {
    if (findPath(maze, current_X + 1, current_Y, exit_X, exit_Y, LEFT) == 1)
    {
      maze[current_Y][current_X + 1] = 2;
      return 1;
    }
  }
  if (maze[current_Y + 1][current_X] == 0 && dir != BOTTOM) {
    if (findPath(maze, current_X, current_Y + 1, exit_X, exit_Y, TOP) == 1)
    {
      maze[current_Y + 1][current_X] = 2;
      return 1;
    }
  }
  if (maze[current_Y - 1][current_X] == 0 && dir != TOP) {
    if (findPath(maze, current_X, current_Y - 1, exit_X, exit_Y, BOTTOM) == 1)
    {
      maze[current_Y - 1][current_X] = 2;
      return 1;
    }
  }

  // If we are in the dead-end, then the way was wrong, so we chage value from 3 to 0, using backtraking.
  maze[current_Y][current_X] = 0;
  return 0;
}

int main()
{

  findPath(maze, 0, 26, 48, 39, TOP);

  
  for (int i = 0; i < HEIGHT; i++)
  {
    for (int j = 0; j < WIDTH; j++)
    {
      if (maze[i][j] == 1)
      {
        red();
        printf(" # ");
      } else if (maze[i][j] == 0)
      {
        yellow();
        printf("   ");
      }
      else
      {
        green();
        printf(" P ");
      }
    }
    printf("\n");
  }
  
  return 0;
}
