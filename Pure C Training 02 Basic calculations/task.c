#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

void equation1 (float a) 
{
  float result = (sin(2 * a) + sin(5 * a) - sin(3 * a)) / (cos(a) + 1 - 2 * pow(sin(a), 2));

  printf("\nEquation 1 result: %f", result);
}

void equation2 (float a)
{
  float result = 2 * sin(a);

  printf("\nEquation 2 result: %f", result);
}

float randNumInRange (double range)
{
  return (float)(((double)rand()/(double)(RAND_MAX)) * range * 2) - (float)range;
}

int main()
{
  srand(time(NULL));

  for (int i = 0; i < 5; ++i)
  {
    float a = randNumInRange(10000);

    printf("\nRun %d", i + 1);
    printf("\narg: %f", a);
    
    equation1(a);
    equation2(a);
  }

  printf("\n\n");
  return 0;
}
