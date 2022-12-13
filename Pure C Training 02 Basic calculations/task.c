#include <stdio.h>
#include <math.h>

void equation1 (double a) 
{
  double result = (sin(2 * a) + sin(5 * a) - sin(3 * a)) / (cos(a) + 1 - 2 * pow(sin(a), 2));

  printf("\nEquation 1 result: %f", result);
}

void equation2 (double a)
{
  double result = 2 * sin(a);

  printf("\nEquation 2 result: %f", result);
}

int main()
{

  for (int i = 1; i <= 5; ++i)
  {
    printf("\nRun %d", i);

    equation1(i);
    equation2(i);
  }


  return 0;
}
