/*
 * Copyright [2015] [Ke Sun <sunke.polyu@gmail.com>]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mocap_base/Subject.h>

namespace mocap {

  Subject::Subject(ros::NodeHandle* nptr, const std::string& sub_name,
                   const std::string& p_frame):
    publish_raw  (false),
    name         (sub_name),
    status       (LOST),
    nh_ptr       (nptr),
    parent_frame (p_frame)
  {
    last_measurement_time = ros::Time::now().toSec();
    pub_filter = nh_ptr->advertise<nav_msgs::Odometry>(name+"/odom", 10);
    if(publish_raw)
      raw_pub = nh_ptr->advertise<nav_msgs::Odometry>(name+"/raw_odom", 10);
    return;
  }

  // Get and set name of the subject
  const std::string& Subject::getName() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return name;
  }
  void Subject::setName(const std::string& sub_name) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    name = sub_name;
  }

  // Enable or diable the subject
  const Subject::Status& Subject::getStatus() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return status;
  }
  void Subject::enable() {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    status = INITIALIZING;
  }
  void Subject::disable() {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    kFilter.reset();
    status = LOST;
  }

  // Get the state of the subject
  const Eigen::Quaterniond& Subject::getAttitude() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return kFilter.attitude;
  }
  const Eigen::Vector3d& Subject::getPosition() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return kFilter.position;
  }
  const Eigen::Vector3d& Subject::getAngularVel() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return kFilter.angular_vel;
  }
  const Eigen::Vector3d& Subject::getLinearVel() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return kFilter.linear_vel;
  }

  // Set the noise parameter for the kalman filter
  bool Subject::setParameters(
                              const Eigen::Matrix<double, 12, 12>& u_cov,
                              const Eigen::Matrix<double, 6, 6>& m_cov,
                              const int& freq) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    return kFilter.init(u_cov, m_cov, freq);
  }

  // Return time since last measurement
  double Subject::timeSinceLastMeasurement()
  {
    return ros::Time::now().toSec()-last_measurement_time;
  }

  // Process the new measurement
  void Subject::processNewMeasurement(
                                      const double& time,
                                      const Eigen::Quaterniond& m_attitude,
                                      const Eigen::Vector3d& m_position) {

    boost::unique_lock<boost::shared_mutex> write_lock(mtx);

    if (!kFilter.isReady()) {
      status = INITIALIZING;
      kFilter.prepareInitialCondition(time, m_attitude, m_position);
      return;
    }

    status = TRACKED;
    // Perfrom the kalman filter
    kFilter.prediction(time);
    kFilter.update(m_attitude, m_position);

    last_measurement_time = time;

    // Publish the new state
    nav_msgs::Odometry odom_filter;

    odom_filter.header.stamp = ros::Time(time);
    odom_filter.header.frame_id = parent_frame;
    odom_filter.child_frame_id = name;

    if(publish_raw){
      tf::quaternionEigenToMsg(m_attitude, odom_filter.pose.pose.orientation);
      tf::pointEigenToMsg(kFilter.position, odom_filter.pose.pose.position);
      tf::vectorEigenToMsg(kFilter.angular_vel, odom_filter.twist.twist.angular);
      tf::vectorEigenToMsg(kFilter.linear_vel, odom_filter.twist.twist.linear);

      raw_pub.publish(odom_filter);
    }

    tf::quaternionEigenToMsg(kFilter.attitude, odom_filter.pose.pose.orientation);
    tf::pointEigenToMsg(kFilter.position, odom_filter.pose.pose.position);
    tf::vectorEigenToMsg(kFilter.angular_vel, odom_filter.twist.twist.angular);
    tf::vectorEigenToMsg(kFilter.linear_vel, odom_filter.twist.twist.linear);
    // To be compatible with the covariance in ROS, we have to do some shifting
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > pose_cov(odom_filter.pose.covariance.begin());
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > vel_cov(odom_filter.twist.covariance.begin());
    pose_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 3);
    pose_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(3, 0);
    pose_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 3);
    pose_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(0, 0);
    vel_cov.topLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 9);
    vel_cov.topRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(9, 6);
    vel_cov.bottomLeftCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 9);
    vel_cov.bottomRightCorner<3, 3>() = kFilter.state_cov.block<3, 3>(6, 6);

    pub_filter.publish(odom_filter);

    return;
  }
}
