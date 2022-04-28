#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "JackalStatePropagator.h"
#include <stdio.h>



JackalStatePropagator::JackalStatePropagator(ompl::control::SpaceInformationPtr si) : StatePropagator(si){
      ROS_INFO("Jackal state prop constructor\n");
}

JackalStatePropagator::~JackalStatePropagator(){
}



//I fixed this too.
void JackalStatePropagator::convert_to_model_space(const double *planner_state, float *model_state){
  // 0 1 2   3  4  5    6  7  8  9    10   11 12 13   14 15 16   17  18  19  20
  // x,y,z,  qx,qy,qz,  q1,q2,q3,q4,  qw,  vx,vy,vz,  ax,ay,az,  qd1,qd2,qd3,qd4
  model_state[4] = planner_state[0];
  model_state[5] = planner_state[1];
  model_state[6] = planner_state[2];

  model_state[0] = planner_state[3];  //qx
  model_state[1] = planner_state[4];  //qy
  model_state[2] = planner_state[5];  //qz
  model_state[3] = planner_state[6];  //qw

  model_state[7] = 0;
  model_state[8] = 0;
  model_state[9] = 0;
  model_state[10] = 0;

  model_state[14] = planner_state[7]; //vx
  model_state[15] = planner_state[8]; //vy
  model_state[16] = planner_state[9]; //vz

  model_state[11] = planner_state[10]; //wx
  model_state[12] = planner_state[11]; //wy
  model_state[13] = planner_state[12]; //wz

  model_state[17] = planner_state[13]; //qd1
  model_state[18] = planner_state[14]; //qd2
  model_state[19] = planner_state[15]; //qd3
  model_state[20] = planner_state[16]; //qd4
}

//I fixed this.
void JackalStatePropagator::convert_to_planner_space(double *planner_state, const float *model_state){
  planner_state[0] = model_state[4]; //x
  planner_state[1] = model_state[5]; //y
  planner_state[2] = model_state[6]; //z

  planner_state[3] = model_state[0];  //qx
  planner_state[4] = model_state[1];  //qy
  planner_state[5] = model_state[2];  //qz
  planner_state[6] = model_state[3]; //qw

  planner_state[7] = model_state[14]; //vx
  planner_state[8] = model_state[15]; //vy
  planner_state[9] = model_state[16]; //vz

  planner_state[10] = model_state[11]; //wx
  planner_state[11] = model_state[12]; //wy
  planner_state[12] = model_state[13]; //wz

  planner_state[13] = model_state[17]; //qd1
  planner_state[14] = model_state[18]; //qd2
  planner_state[15] = model_state[19]; //qd3
  planner_state[16] = model_state[20]; //qd4
}



void JackalStatePropagator::propagate(const ompl::base::State *state, const ompl::control::Control *control, double duration, ompl::base::State *result) const{
  const double* val = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* result_val = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  
  const double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  
  float x_start[21];
  float x_end[21];
  
  HybridDynamics solver;
  solver.initState((float *) x_start);
  
  convert_to_model_space(val, x_start);
  for(int i = 0; i < solver.STATE_DIM; i++){
    solver.state_[i] = x_start[i];
  }
  
  float Vf = control_vector[1];//GlobalParams::get_fuzzy_constant_speed();
  float Wz = control_vector[0];
  float base_width = 2*Jackal::rcg::tx_front_left_wheel;
  float vl = (Vf - Wz*(base_width/2.0))/Jackal::rcg::tire_radius;
  float vr = (Vf + Wz*(base_width/2.0))/Jackal::rcg::tire_radius;
  
  
  //ROS_INFO("Solver control: <%f %f>", control_vector[0], control_vector[1]);
  //ROS_INFO("Solver start: <%f %f>     control: <%f %f>     <%f %f>", x_start[0], x_start[1],   control_vector[0], control_vector[1],   vl, vr);
  //ROS_INFO("Duration %f", duration);
  
  for(int i = 0; (i*solver.stepsize) < duration; i++){
    solver.step(vl, vr);
  }
  
  //ROS_INFO("Solver end: <%f %f>\n", x_end[0], x_end[1]);
  
  //These conversions are bandaids and spaghetti code
  for(int i = 0; i < solver.STATE_DIM; i++){
    x_end[i] = solver.state_[i];
  }
  
  convert_to_planner_space(result_val, x_end);
}



bool JackalStatePropagator::steer(const ompl::base::State *from, const ompl::base::State *to, ompl::control::Control *result, double &duration) const{
  return false;
  /*
  const double* start_state = from->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  const double* goal_state = to->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* result_state = result->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  //ignore this.
  double *control_vector = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
  control_vector[0] = 0;
  control_vector[1] = 0; //Not interested.
  
  float X_start[21];
  float X_end[21];
  float X_best[21];
  float best_err = 1000000;
  float err = 1;
  
  float x_goal = goal_state[0];
  float y_goal = goal_state[1];
  
  JackalDynamicSolver solver;
  convert_to_model_space(val, X_start);

  float base_width = solver.base_size[0];
  float Vf;
  float Wz;
  
  std::vector<Vector2f> waypoints;
  waypoints.push_back(Vector2f(X_start[0], X_start[1]));
  waypoints.push_back(Vector2f(X_end[0], X_end[1]));
  waypoints.push_back(Vector2f(X_end[0], X_end[1]));

  geometry_msgs::Pose pose;
  pose.position.x = X_start[0];
  pose.position.y = X_start[1];
  pose.position.z = X_start[2];

  pose.orientation.x = X_start[3];
  pose.orientation.y = X_start[4];
  pose.orientation.z = X_start[5];
  pose.orientation.w = X_start[10];

  float timestep = .1f; //this is like the rate at which the control system gets called.
  float elapsed_time = 0;
  
  do{
    wayypoints[0][0] = X_start[0];
    wayypoints[0][1] = X_start[1];
    control_system_->computeVelocityCommand(waypoints, pose, Vf, Wz);
    
    float vl = (Vf - Wz*(base_width/2.0))/solver.tire_radius;
    float vr = (Vf + Wz*(base_width/2.0))/solver.tire_radius;
    
    solver.solve(X_start, X_end, vl, vr, timestep);

    float dx = X_end[0] - x_goal;
    float dy = X_end[1] - y_goal;
    
    err = sqrtf((dx*dx)+(dy*dy));
    if(err < best_err){
      best_err = err;
      for(unsigned i = 0; i < 21; i++){
        X_best[i] = X_end[i];
      }
    }

    for(unsigned i = 0; i < 21; i++){
      X_start[i] = X_end[i];
    }

    elapsed_time += timestep;
  } while(err > .1 && elapsed_time < duration);

  ROS_INFO("best_err");

  convert_to_planner_space(result_val, X_best);
  duration = elapsed_time;
  return false;
  */
}

bool JackalStatePropagator::canPropagateBackward() const{
  return false;
}

bool JackalStatePropagator::canSteer() const{ //Steering means, can we drive exactly from one state to the next and compute the control necessary to do it. I think.
  return false;
}
