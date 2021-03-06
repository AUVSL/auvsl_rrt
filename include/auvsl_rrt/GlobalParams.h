#include <ros/ros.h>
#include <vector>


#pragma once

class GlobalParams{
public:
  static void load_params(ros::NodeHandle *nh);
  static float get_fuzzy_constant_speed();
  static float get_max_angular_vel();
  static float get_timestep();
  static int get_debug_level();
  static float get_planner_resolution();
  static float get_state_checker_resolution();
  static float get_propagation_step_size();
  static float get_visualize_planner();

  static float get_goal_bias();
  static float get_goal_bias2();

  static std::vector<float> get_distance_weights();
  static int get_num_control_samples();
  static int get_seed();
  static bool get_add_intermediate_states();

  static float get_heading_heuristic();
  static float get_forward_variance();
  static float get_angular_variance();

  static float get_p_gain();
  static float get_goal_tolerance();
  static float get_max_gp_runtime();

private:
  static float fuzzy_constant_speed;
  static float max_angular_vel;
  static float timestep;
  static int debug_level;
  static float planner_resolution;
  static float state_checker_resolution;
  static float propagation_step_size;
  static float visualize_planner;

  static float goal_bias;
  static float goal_bias2;

  static std::vector<float> distance_weights;
  static int num_control_samples;
  static bool add_intermediate_states;
  static int seed;

  static float heading_heuristic;
  static float angular_variance;
  static float forward_variance;

  static float p_gain;

  static float goal_tolerance;
  static float max_gp_runtime;
};
