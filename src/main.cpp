#include "InvertedPendulum.hpp"
#include <math.h>
#include <iostream>

int main()
{
  // create the Inverted Pedulum Model
  InvertedPendulum model;

  // define the initial state of the model
  std::vector<double> x0(4);
  x0[0] = -0.0;     // x
  x0[1] = 0.0;      // x'
  x0[2] = 0;   // theta
  x0[3] = 0;        // theta'
  model.initialize(x0);

  // start simulation in seperate thread
  double dt = 0.03;
  std::thread simulation_thread(&InvertedPendulum::simulate, &model, dt); 

  // test the force interface by setting forces in while loop! 
  while(true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(850));
    model.setForce(0.3);
    std::this_thread::sleep_for(std::chrono::milliseconds(850)); 
    model.setForce(-0.3);
  }

  // script comes to an end, wait for threads to be finished
  simulation_thread.join();
  return 0; 
}