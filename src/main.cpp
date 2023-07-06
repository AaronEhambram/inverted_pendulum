#include "InvertedPendulum.hpp"
#include <math.h>

int main()
{
  // create the Inverted Pedulum Model
  InvertedPendulum model;

  // define the initial state of the model
  std::vector<double> x0(4);
  x0[0] = -0.0;     // x
  x0[1] = 0.0;      // x'
  x0[2] = M_PI/2;   // theta
  x0[3] = 0;        // theta'
  model.initialize(x0);

  // start simulation in seperate thread
  double dt = 0.001;
  std::thread simulation_thread(&InvertedPendulum::simulate, model, dt); 

  // TODO: test the force interface by setting forces in while loop! 

  // script comes to an end, wait for threads to be finished
  simulation_thread.join();
  return 0; 
}