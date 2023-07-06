#include <vector>
#include <thread>
#include <mutex>
#include "opencv2/core.hpp"

class InvertedPendulum
{

  public:
  InvertedPendulum(); 
  void setForce(const double& f);
  void setDisturbanceForce(const double& s);
  void initialize(const std::vector<double>& x0);  
  void simulate(double dt); 
  // defines the ordinary differential equation: x' = f(x) 
  void operator() (const std::vector<double> &x, std::vector<double> &dxdt, double t);
  enum WallCollisionModel{SPRING,ELASTIC_COLLISION};

  private:

  // Model parameters
  double M = 0.4; // mass of the cart
  double m = 0.2; // mass of the pendulum
  double l = 0.3; // half length of the pendulum
  double I; // moment of inertia of the pendulum -> calculated
  double b = 0.5; // friction coefficient
  double b_rot = 0.01; // friction coefficient rotation
  double g = 9.81; // gravity
  double F = 0.0; // Force on the cart
  double l_track = 1.0; // length of the track the cart is mounted on
  double S = 0.0; // Disturbance force
  WallCollisionModel wall_collision_model = ELASTIC_COLLISION;
  double time_step_length; 

  // internal state of the pendulum
  std::vector<double> state; 

  // model ok check -> only simulate if ok
  bool model_ok = true; 

  // thread-safety interface for F and S when set by user
  std::shared_ptr<std::mutex> mutex_S_ptr;
  std::shared_ptr<std::mutex> mutex_F_ptr;

  // Visualization attributes
  cv::Mat base_im; 
  double pix_per_m = 700; 
  std::string window_name = "Inverted Pendulum"; 
  cv::Point2d origin_in_image; 

  // print and visualization functions
  void print_state(); 
  void create_base_image(); 
  void draw_pendulum(cv::Mat& im); 
  void w2pix(const cv::Point2d& p_w, cv::Point2d& p_pix);
};