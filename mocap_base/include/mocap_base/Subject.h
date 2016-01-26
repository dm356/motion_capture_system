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

#ifndef SUBJECT_H
#define SUBJECT_H


#include <map>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mocap_base/KalmanFilter.h>

namespace mocap{

/*
 * @brief Subject Defines attributes of a rigid body
 */
class Subject {
  public:
    typedef boost::shared_ptr<Subject> Ptr;
    typedef const boost::shared_ptr<Subject> ConstPtr;
    typedef std::map<std::string,Ptr> Map;

    enum Status {
      LOST,
      INITIALIZING,
      TRACKED
    };

    /*
     * @brief Constructor and Destructor
     */
    Subject(ros::NodeHandle* nptr, const std::string& sub_name,
        const std::string& p_frame);
    ~Subject() {}

    /*
     * @brief getName setName
     *    Get and set the name of the object
     */
    const std::string& getName();
    void setName(const std::string& sub_name);
    /*
     * @brief isActive Tells if the object is still active or not
     */
    const Status& getStatus();
    void enable();
    void disable();
    /*
     * @brief getAttitude getPosition getAngularVel getLinearVel
     *    Returns the state of the object
     */
    const Eigen::Quaterniond& getAttitude();
    const Eigen::Vector3d& getPosition();
    const Eigen::Vector3d& getAngularVel();
    const Eigen::Vector3d& getLinearVel();

    /*
     * @brief setNoiseParameter Set noise parameters for
     *    the kalman filter
     * @param u_cov Input noise
     * @param m_cov Measurement noise
     * @return True if success
     */
    bool setParameters(
        const Eigen::Matrix<double, 12, 12>& u_cov,
        const Eigen::Matrix<double, 6, 6>& m_cov,
        const int& freq);

    /**
     * @brief timeSinceLastMeasurement Get number of seconds that have passed since the last measurement was received
     *
     * @return Time in seconds
     */
    double timeSinceLastMeasurement();

    /*
     * @brief processNewMeasurement Process new measurements
     *    from the mocap system
     * @param m_attitude Measured attitude
     * @param m_position Measured position
     */
    void processNewMeasurement(
        const double& time,
        const Eigen::Quaterniond& m_attitude,
        const Eigen::Vector3d& m_position);

    bool publish_raw;

  private:
    // Disable copy constructor and assign operator
    Subject(const Subject&);
    Subject& operator=(const Subject&);

    // Name of the subject
    std::string name;

    // Error state Kalman filter
    KalmanFilter kFilter;

    // Tells the status of the object
    Status status;

    // Prevent cocurrent reading and writing of the class
    boost::shared_mutex mtx;

    double last_measurement_time;

    // Publisher for the subject
    ros::NodeHandle* nh_ptr;
    std::string parent_frame;
    ros::Publisher pub_filter;
    ros::Publisher raw_pub;
};
}


#endif
