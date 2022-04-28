#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>

#include "GlobalParams.h"
#include "PlannerVisualizer.h"
#include "GlobalPlanner.h"
#include <auvsl_dynamics/HybridDynamics.h>
#include "OctoTerrainMap.h"


#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <rbdl/rbdl.h>

#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>


unsigned got_init_pose = 0;
geometry_msgs::Pose initial_pose;

unsigned got_grid_map = 0;
geometry_msgs::Pose origin;
float map_res;
unsigned height;
unsigned width;

OctoTerrainMap *terrain_map;



void get_pos_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  initial_pose = msg->pose.pose;
  got_init_pose = 1;
}

float evaluateSimulatedTrajectory(){
  //Step 1. Read Odometry into an array for searching.
  ROS_INFO("Evaluating");
  std::ifstream odom_file("/home/justin/code/AUVSL_ROS/bags/rantoul/bags/rantoul_2_odom_10hz.csv");
  std::string line;
  std::stringstream ss;
  char comma;
  unsigned num_rows = 0;
  float *X_odom;

  ROS_INFO("Reading File");
  while(odom_file.peek() != EOF){
    std::getline(odom_file, line);
    num_rows++;
  }
  
  odom_file.clear();
  odom_file.seekg(0);
  
  X_odom = new float[3*num_rows];

  unsigned timestamp;
  ROS_INFO("X_odom");
  for(unsigned i = 0; i < num_rows; i++){
    std::getline(odom_file, line);
    ss.str(line);
    ss.clear();

    unsigned row = 3*i;

    ss >> timestamp >> comma;
    ss >> X_odom[row+0] >> comma;
    ss >> X_odom[row+1] >> comma;
    ss >> X_odom[row+2];// >> comma;
  }
  
  
  ROS_INFO("sim xout file");
  //Step 2. Open and prepare xout.csv to be read.
  std::ifstream sim_xout_file("/home/justin/xout_file.csv");
  std::getline(sim_xout_file, line); //remove first line
  
  float sim_x, sim_y, sim_z, ignoreme;

  ROS_INFO("parse sim xou file");
  while(sim_xout_file.peek() != EOF){
    std::getline(sim_xout_file, line);
    ss.str(line);
    ss.clear();
    
    ss >> ignoreme >> comma; //skip quaternion components
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    
    ss >> sim_x >> comma;
    ss >> sim_y >> comma;
    ss >> sim_z;  // >> comma;
    
  }
  
  ROS_INFO("dxdy");
  float dx = sim_x - X_odom[(3*(num_rows-1))+0];
  float dy = sim_y - X_odom[(3*(num_rows-1))+1];
  
  delete[] X_odom;
  return sqrtf((dx*dx) + (dy*dy));
}

void record_altitude_under_path(TerrainMap *terrain_map){
  std::string line;
  std::stringstream ss;
  char comma;
  std::ifstream sim_xout_file("/home/justin/xout_file.csv");
  std::getline(sim_xout_file, line);
  
  float sim_x, sim_y, sim_z, ignoreme, alt;
  std::ofstream alt_log("/home/justin/alt_file.csv");
  alt_log << "r,alt\n";

  alt = 0;
  
  while(sim_xout_file.peek() != EOF){
    std::getline(sim_xout_file, line);
    ss.str(line);
    ss.clear();
    
    ss >> ignoreme >> comma; //skip quaternion components
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    ss >> ignoreme >> comma;
    
    ss >> sim_x >> comma;
    ss >> sim_y >> comma;
    ss >> sim_z;   // >> comma;
    
    alt = terrain_map->getAltitude(sim_x, sim_y, alt);
    alt_log << sqrtf(sim_x*sim_x + sim_y*sim_y) << ',' << alt << "\n";
    
  }
  
  sim_xout_file.close();
  alt_log.close();
  
}

int main(int argc, char **argv){
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  ROS_INFO("Starting up test_terrain_node\n");
  
  int plot_res;
  nh.getParam("/TerrainMap/plot_res", plot_res); 
  
  GlobalParams::load_params(&nh);
  ompl::RNG::setSeed(GlobalParams::get_seed());
  srand(GlobalParams::get_seed());

  ros::Rate loop_rate(10);
  
  ROS_INFO("Getting OctoTerrainMap");
  std::string site_cloud_fn;
  nh.getParam("/TerrainMap/site_cloud_filename", site_cloud_fn);
  terrain_map = new OctoTerrainMap(site_cloud_fn.c_str());
  //SimpleTerrainMap simple_terrain_map;
  ROS_INFO("Constructed terrain map");
  
  //ros::spin();
  
  /*
  std::ofstream log_file;
  log_file.open("/home/justin/elev.csv", std::ofstream::out);
  log_file << "x,y,alt\n";
  
  float x;
  float y;
  float alt = 0;
  float occ;
  unsigned idx;

  for(int j = 0; j < terrain_map->rows_; j++){
      idx = j*terrain_map->cols_;
      for(int i = 0; i < terrain_map->cols_; i++){
          x = (i*terrain_map->map_res_) + terrain_map->x_origin_;
          y = (j*terrain_map->map_res_) + terrain_map->y_origin_;
          
          alt = terrain_map->getAltitude(x, y, alt); //elev_map_[idx+i];
          log_file << x << "," << y << "," << alt << "\n";
      }
   }
   
   
  // int i,j;
  // for(float x = 0; x < 1; x += .01f){
  //   for(float y = -100; y < -99; y += .01f){
  //     alt = terrain_map->getAltitude(x, y, alt);
  //     log_file << x << "," << y << "," << alt << "\n";
  //   }
  // }
  
  log_file.close();
  */
  
  char ignore;
  float X_final[21];
  float X_start[21];
  
  HybridDynamics solver;
  solver.setAltitudeMap([terrain_map](float x, float y, float z_guess){return terrain_map->getAltitude(x, y, z_guess);});
  solver.initState();
  solver.settle();
  for(int i = 0; i < 200; i++){
    solver.step(10.0f, 10.0f);
  }
  ROS_INFO("Final Position %f %f", solver.state_[0], solver.state_[1]);
  
  //record_altitude_under_path((TerrainMap*) terrain_map);
  
  ROS_INFO("test_terrain is exiting");
  
  return 0;
}
