#include <stdio.h>
#include <stdlib.h>
#include "maze.h"

typedef enum direction 
{
  TOP,
  LEFT,
  RIGHT,
  BOTTOM
} direction;

typedef struct findPathArgs 
{
  int current_X;
  int current_Y;
  int exit_X;
  int exit_Y;
  direction dir;
} findPathArgs;

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
int findPath (struct findPathArgs *args) 
{
  //printf("%d\n x: %d\n y: %d\n", maze[args->current_Y][args->current_X], args->current_X + 1, args->current_Y + 1);
  // Checking if current point is our destination
  if (args->current_X == args->exit_X && args->current_Y == args->exit_Y) 
  {
    return 1;
  }

  // Changing current position to 3, to mark passed ways and avoid infinite loops
  maze[args->current_Y][args->current_X] = 3;

  // To see full algorithm path (x, y format)


  // Depending from where it came, choose which neigbours must be scanned. Never scannes were it came from.
  // Basically moving our position in all possible ways to find a destination. 
  // If all possible available coordinates were checked returns 0.
  // When we arrive at our destination we change the path values to 2, using backtracking.
  if (maze[args->current_Y][args->current_X - 1] == 0 && args->dir != LEFT) {
    args->current_X -= 1;
    args->dir = RIGHT;
    if (findPath(args) == 1)
    {
      maze[args->current_Y][args->current_X] = 2;
      args->current_X += 1;
      return 1;
    }
    args->current_X += 1;
    args->dir = LEFT;
  }
  if (maze[args->current_Y][args->current_X + 1] == 0 && args->dir != RIGHT) {
    args->current_X += 1;
    args->dir = LEFT;
    if (findPath(args) == 1)
    {
      maze[args->current_Y][args->current_X] = 2;
      args->current_X -= 1;
      return 1;
    }
    args->current_X -= 1;
    args->dir = RIGHT;
  }
  if (maze[args->current_Y + 1][args->current_X] == 0 && args->dir != BOTTOM) {
    args->current_Y += 1;
    args->dir = TOP;
    if (findPath(args) == 1)
    {
      maze[args->current_Y][args->current_X] = 2;
      args->current_Y -= 1;
      return 1;
    }
    args->current_Y -= 1;
    args->dir = BOTTOM;
  }
  if (maze[args->current_Y - 1][args->current_X] == 0 && args->dir != TOP) {
    args->current_Y -= 1;
    args->dir = BOTTOM;
    if (findPath(args) == 1)
    {
      maze[args->current_Y][args->current_X] = 2;
      args->current_Y += 1;
      return 1;
    }
    args->current_Y += 1;
    args->dir = TOP;
  }

  // If we are in the dead-end, then the way was wrong, so we chage value from 3 to 0, using backtraking.
  maze[args->current_Y][args->current_X] = 0;
  return 0;
}

int main()
{
  struct findPathArgs args;
  args.current_X = 0;
  args.current_Y = 26;
  args.exit_X = 49;
  args.exit_Y = 9;
  args.dir = LEFT;
  
  findPath(&args);


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
