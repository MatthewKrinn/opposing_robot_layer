#include <opposing_robot_layer/opposing_robot_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_math.h>

PLUGINLIB_EXPORT_CLASS(opposing_robot_layer::OpposingRobotLayer, costmap_2d::Layer)

namespace opposing_robot_layer
{

OpposingRobotLayer::OpposingRobotLayer() : has_data_(false) {}

void OpposingRobotLayer::onInitialize()
{
  ROS_INFO("Initializing Opposing Robot Layer...");
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  nh.param("enabled", enabled_, true);

  // Subscribe to the opposing robot’s pose with covariance
  subscriber_ = nh.subscribe("/global_ball_data", 1, &OpposingRobotLayer::ballCallback, this);
  ROS_INFO("DONE INITIALIZNG");
}

void OpposingRobotLayer::ballCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  covariance_x_ = msg->pose.covariance[0];  // variance in x
  covariance_y_ = msg->pose.covariance[7];  // variance in y
  has_data_ = true;
}

void OpposingRobotLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_ || !has_data_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);

  double radius = std::sqrt(covariance_x_ + covariance_y_) * 2.0;

  *min_x = std::min(*min_x, robot_x_ - radius);
  *min_y = std::min(*min_y, robot_y_ - radius);
  *max_x = std::max(*max_x, robot_x_ + radius);
  *max_y = std::max(*max_y, robot_y_ + radius);
}

void OpposingRobotLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                     int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !has_data_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);

  unsigned int mx, my;
  if (!master_grid.worldToMap(robot_x_, robot_y_, mx, my)) return;

  double radius = std::sqrt(covariance_x_ + covariance_y_) * 2.0;
  int cell_radius = static_cast<int>(radius / master_grid.getResolution());

  for (int dx = -cell_radius; dx <= cell_radius; ++dx)
  {
    for (int dy = -cell_radius; dy <= cell_radius; ++dy)
    {
      int nx = mx + dx;
      int ny = my + dy;

      if (nx >= 0 && ny >= 0 &&
          nx < static_cast<int>(master_grid.getSizeInCellsX()) &&
          ny < static_cast<int>(master_grid.getSizeInCellsY()))
      {
        double dist = std::sqrt(dx * dx + dy * dy) * master_grid.getResolution();
        unsigned char old_cost = master_grid.getCost(nx, ny);

        // 🔴 Lethal zone: direct obstacle
        if (dist <= radius * 0.3)
        {
          if (old_cost < costmap_2d::LETHAL_OBSTACLE)
            master_grid.setCost(nx, ny, costmap_2d::LETHAL_OBSTACLE);
        }
        // 🟠 Buffer zone: soft inflation area
        else if (dist <= radius)
        {
          if (old_cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            master_grid.setCost(nx, ny, costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
        }
      }
    }
  }
}

} // namespace opposing_robot_layer
