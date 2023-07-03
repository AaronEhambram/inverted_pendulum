#include "InvertedPendulum.hpp"
#include <math.h>

int main()
{
  InvertedPendulum model;
  std::vector<double> x0(4);
  x0[0] = -0.6; x0[1] = 2.0; x0[2] = 0;  x0[3] = 0;   
  model.initialize(x0);
  model.simulate(0.001);  
  return 0; 
}