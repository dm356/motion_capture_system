/*
 * Copyright [2015] [Derek Mitchell <dmitchell356@gmail.com>]
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

#ifndef BATCH_HANDLER_H
#define BATCH_HANDLER_H


#include <map>
#include <string>
#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <mocap_base/KalmanFilter.h>
#include <mocap_base/Subject.h>

namespace mocap{

  /*
   * @brief BatchHandler Defines attributes of a rigid body
   */
  class BatchHandler {
  public:
    typedef boost::shared_ptr<BatchHandler> Ptr;
    typedef const boost::shared_ptr<BatchHandler> ConstPtr;
    typedef std::vector<Ptr> Vec;
    typedef std::map<std::string,Ptr> Map;

    /*
     * @brief Constructor and Destructor
     */
    BatchHandler(ros::NodeHandle* nptr, const std::string& sub_name, const std::string& p_frame, double timeout = 0);
    ~BatchHandler() {}

    /*
     * @brief getName setName
     *    Get and set the name of the object
     */
    const std::string& getName();
    void setName(const std::string& sub_name);

    /*
     * @brief setParameters Set noise parameters for
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
     * @brief addMeasurement Store measurement before distributing to Subjects
     *
     * @param time Time measurement was taken
     * @param m_attitude Measured attitude
     * @param m_position Measured position
     */
    void addMeasurement(const double& time, const Eigen::Quaterniond& m_attitude, const Eigen::Vector3d& m_position);


    /**
     * @brief processMeasurements Distribute stored measurements to active objects
     */
    void processMeasurements();

  private:

    /**
     * @brief Measurement Temporarily holds measurements before passing on to the proper Subject
     */
    struct Measurement
    {
      typedef boost::shared_ptr<Measurement> Ptr;

      Measurement(const double& m_time, const Eigen::Quaterniond& m_attitude, const Eigen::Vector3d& m_position):time(m_time),attitude(m_attitude),position(m_position){}

      double time;
      Eigen::Quaterniond attitude;
      Eigen::Vector3d position;
    };

    // Disable copy constructor and assign operator
    BatchHandler(const BatchHandler&);
    BatchHandler& operator=(const BatchHandler&);

    // Name of the subject
    std::string name;

    // Prevent cocurrent reading and writing of the class
    boost::shared_mutex mtx;

    // Publisher for the subject
    ros::NodeHandle* nh_ptr;
    std::string parent_frame;

    // Count of objects activated since start
    int object_count;

    // Time before object is considered lost
    double measurement_timeout;

    // Kalman filter params
    Eigen::Matrix<double, 12, 12> proc_cov;
    Eigen::Matrix<double, 6, 6> meas_cov;
    int frequency;

    // Vector of currently active subjects
    std::vector<Subject::Ptr> subjects;
    std::vector<Measurement::Ptr> measurements;
  };
}


#endif
