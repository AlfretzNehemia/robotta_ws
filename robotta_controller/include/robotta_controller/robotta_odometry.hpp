#ifndef _ROBOTTA__ODOMETRY_HPP_
#define _ROBOTTA__ODOMETRY_HPP_

#include <cmath>

#include "robotta_controller/rolling_mean_accumulator.hpp"
#include "rclcpp/time.hpp"

namespace robotta
{
namespace controller
{
class RobottaOdometry
{
public:
  explicit RobottaOdometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double left_pos, double right_pos, const rclcpp::Time & time);
  void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
  void resetOdometry();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinear() const { return linear_; }
  double getAngular() const { return angular_; }

  void setWheelParams(double wheel_separation, double left_wheel_radius, double right_wheel_radius);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = robotta::controller::RollingMeanAccumulator<double>;

  void integrateRungeKutta2(double linear, double angular);
  void integrateExact(double linear, double angular);
  void resetAccumulators();

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;  // [rad]

  // Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  // Wheel kinematic parameters [m]:
  double wheel_separation_;
  double left_wheel_radius_;
  double right_wheel_radius_;

  // Previous wheel position/state [rad]:
  double left_wheel_old_pos_;
  double right_wheel_old_pos_;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace controller
}

#endif  // _ROBOTTA__ODOMETRY_HPP_