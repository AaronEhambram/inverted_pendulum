#include "InvertedPendulum.hpp"
#include <math.h>
#include <chrono>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

InvertedPendulum::InvertedPendulum()
{
  // compute the moment of Inertia of the rod
  I = 4.0/3.0 * m * pow(l,2); // l is half of the rod's size!
  state.resize(4);
  create_base_image(); 
  cv::namedWindow(window_name); 
  cv::moveWindow(window_name, 3900,60);
}

void InvertedPendulum::setForce(const double& f)
{
  F = f; 
}

void InvertedPendulum::initialize(const std::vector<double>& x0)
{
  state = x0; 
}

void InvertedPendulum::operator()(const std::vector<double> &x, std::vector<double> &dxdt, double t)
{
  /*
  * x = (x, x', theta, theta')
  */
 std::vector<double> x_ = x; 
  
  dxdt[0] = x_[1];
  dxdt[2] = x_[3]; // theta'
  double m2 = pow(m,2);
  double l2 = pow(l,2); 
  double nom = F - b*x_[1] + m*l*pow(x_[3],2)*sin(x_[2]) + m2*g*l2/(I+m*l2)*cos(x_[2])*sin(x_[2]) + b_rot*m*l/(I+m*l2)*x_[3]*cos(x_[2]); 
  double denom = M + m - m2*l2/(I+m*l2)*pow(cos(x_[2]),2); 

  // in case of collision
  if(wall_collision_model == SPRING)
  {
    if(x[0] >= l_track/2.0) // cart has left most position -> cart has to stop here
    {
      nom = nom + (l_track/2.0 - x[0])*100000;
    }
    else if(x[0] <= -l_track/2.0) // cart has right most position -> cart has to stop here
    {
      nom = nom + (-l_track/2.0 - x[0])*100000;
    }
  }
  else if(wall_collision_model == ELASTIC_COLLISION)
  {
    if(x[0] >= l_track/2.0 && x_[1] > 0) // cart has right most position -> cart has to stop here
    {
      nom = std::min(nom,0.0) - 2*M*abs(x_[1])/(time_step_length);
    }
    else if(x[0] <= -l_track/2.0 && x_[1] < 0) // cart has left most position -> cart has to stop here
    {
      nom = std::max(nom,0.0) + 2*M*abs(x_[1])/(time_step_length);
    }
  }
  

  dxdt[1] = nom/denom; // x''
  dxdt[3] = -m*l/(I+m*l2) * dxdt[1] * cos(x_[2]) - m*g*l/(I+m*l2) * sin(x_[2]) - b_rot/(I+m*l2)*x_[3]; // theta''

  
}

void InvertedPendulum::simulate(double dt)
{
  std::chrono::duration<double> sim_step_time(dt*0.8);
  time_step_length = dt; 
  std::thread visu_thread(&InvertedPendulum::print_state,this);
  while(model_ok)
  {
    auto start = std::chrono::system_clock::now();
    size_t steps = boost::numeric::odeint::integrate(*this,state,0.0,dt,dt/15);
    auto intergrate_end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = intergrate_end - start;
    auto sleep_time = sim_step_time - diff;
    std::this_thread::sleep_for(sleep_time);
    /*if(state[0] < l_track/3 && state[0] > -l_track/3)
    {
      if(state[1] > 0)
      {
        setForce(1.8);
      }
      if(state[1] < 0)
      {
        setForce(-1.8);
      }
      
    }
    else
    {
      setForce(0);
    }*/
    /*if(state[0] > l_track/3)
    {
      setForce(0);
    }*/
  }
  visu_thread.join(); 
}

// Visualization functions
void InvertedPendulum::create_base_image()
{
  // generate image with according size
  double cols = (l_track+4.0*l)*pix_per_m+30; // 20 pixel distance to edges
  double rows = 4.0*l*pix_per_m+30; 

  // create the base image
  base_im = cv::Mat(rows,cols,CV_8UC3,cv::Scalar(255,255,255)); 

  // the mode of the base_image is the origin (x to the right, y pointing upward)
  origin_in_image.x = cols/2.0; 
  origin_in_image.y = rows/2.0; 

  // create the line that visualizes the line on which the start point of the pendulum can move
  cv::Point2d left_mid_pix(origin_in_image.x-pix_per_m*0.5*l_track,origin_in_image.y);
  cv::Point2d right_mid_pix(origin_in_image.x+pix_per_m*0.5*l_track,origin_in_image.y);
  cv::line(base_im,left_mid_pix,right_mid_pix,cv::Scalar(0,0,0),2);
}

void InvertedPendulum::w2pix(const cv::Point2d& p_w, cv::Point2d& p_pix)
{
  p_pix.y = origin_in_image.y - p_w.y * pix_per_m;
  p_pix.x = origin_in_image.x + p_w.x * pix_per_m;
}

void InvertedPendulum::draw_pendulum(cv::Mat& im)
{
  // draw the fixation point of the pendulum to the cart
  cv::Point2d pendulum_start_w(state[0],0.0);
  cv::Point2d pendulum_start_pix;
  w2pix(pendulum_start_w, pendulum_start_pix); 
  cv::circle(im,pendulum_start_pix,5,cv::Scalar(0,0,255),2,cv::FILLED);

  // draw the end-point of the pendulum
  cv::Point2d start_to_end_w(2*l*sin(state[2]),-2*l*cos(state[2]));
  cv::Point2d pendulum_end_w = pendulum_start_w + start_to_end_w; 
  cv::Point2d pendulum_end_pix;
  w2pix(pendulum_end_w, pendulum_end_pix); 
  cv::circle(im,pendulum_end_pix,5,cv::Scalar(0,0,255),2,cv::FILLED);

  // draw the pendulum as connecting line between start- and end-point
  cv::line(im,pendulum_start_pix,pendulum_end_pix,cv::Scalar(0,0,255),2);
}

void InvertedPendulum::print_state()
{ 
  // draw the image with pendulum!
  while(model_ok)
  {
    cv::Mat im;
    base_im.copyTo(im); 
    //std::cout << state[0] << " " << state[1] << " " << state[2] << " " << state[3] << std::endl;
    draw_pendulum(im);
    cv::imshow(window_name,im);
    cv::waitKey(1);
  }
}
