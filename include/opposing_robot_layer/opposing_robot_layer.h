#ifndef OPPOSING_ROBOT_LAYER_H_
#define OPPOSING_ROBOT_LAYER_H_

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <mutex>

namespace opposing_robot_layer
{

class OpposingRobotLayer : public costmap_2d::CostmapLayer
{
public:
  OpposingRobotLayer();

  virtual void onInitialize();
  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                           int min_i, int min_j, int max_i, int max_j);

private:
  void ballCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

  std::mutex data_mutex_;
  bool has_data_;
  double robot_x_, robot_y_;
  double covariance_x_, covariance_y_;
  ros::Subscriber subscriber_;
};

} // namespace opposing_robot_layer

#endif
