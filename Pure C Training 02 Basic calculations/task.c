#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

void equation1 (float rad) 
{
  float result = (sin(2 * rad) + sin(5 * rad) - sin(3 * rad)) / (cos(rad) + 1 - 2 * pow(sin(rad), 2));

  printf("\nEquation 1 result: %f", result);
}

void equation2 (float rad)
{
  float result = 2 * sin(rad);

  printf("\nEquation 2 result: %f", result);
}

float randNumInRange (double range)
{
  return (float)(((double)rand()/(double)(RAND_MAX)) * range * 2) - (float)range;
}

int main()
{
  srand(time(NULL));
  float equations_arg;

  for (int i = 0; i < 5; ++i)
  {
    equations_arg = randNumInRange(10000);

    printf("\nRun %d", i + 1);
    printf("\narg: %f", equations_arg);
    
    equation1(equations_arg);
    equation2(equations_arg);
  }

  printf("\n\n");
  return 0;
}
