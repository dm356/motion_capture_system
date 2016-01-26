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

#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mocap_base/BatchHandler.h>

namespace mocap {

  BatchHandler::BatchHandler(ros::NodeHandle* nptr, const std::string& sub_name, const std::string& p_frame, double timeout):
    name         (sub_name),
    nh_ptr       (nptr),
    parent_frame (p_frame),
    object_count(1),
    measurement_timeout(timeout)
  {
    return;
  }

  // Get and set name of the subject
  const std::string& BatchHandler::getName() {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return name;
  }
  void BatchHandler::setName(const std::string& sub_name) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    name = sub_name;
  }

  // Set the noise parameter for the kalman filter
  bool BatchHandler::setParameters(const Eigen::Matrix<double, 12, 12>& u_cov,
                                   const Eigen::Matrix<double, 6, 6>& m_cov,
                                   const int& freq) {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    proc_cov = u_cov;
    meas_cov = m_cov;
    frequency = freq;
    for(auto it=subjects.begin();it!=subjects.end();it++){
      if(!(*it)->setParameters(u_cov,m_cov,freq)){
        return false;
      }
    }
    ROS_INFO("Set parameters for '%s' batch", name.c_str());
    return true;
  }

  // Append measurement to list
  void BatchHandler::addMeasurement(const double& time, const Eigen::Quaterniond& m_attitude, const Eigen::Vector3d& m_position)
  {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    measurements.push_back(boost::make_shared<Measurement>(time,m_attitude,m_position));
    Subject::Ptr s;
    if(subjects.size() < measurements.size())
    {
      s = boost::make_shared<Subject>(nh_ptr,name+std::to_string(object_count),parent_frame);
      s->setParameters(proc_cov,meas_cov,frequency);
      subjects.push_back(s);
      object_count++;
    }
  }

  // Determine mapping of measurements to objects
  void BatchHandler::processMeasurements()
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    size_t N_m = measurements.size(), N_s = subjects.size();
    for(size_t i=0;i<N_s && i<N_m;i++)
    {
      subjects[i]->processNewMeasurement(measurements[i]->time,measurements[i]->attitude,measurements[i]->position);
    }
    measurements.clear();

    double temp = measurement_timeout;
    // Remove inactive objects if batch is not fixed
    if(measurement_timeout > 0){
      subjects.erase(std::remove_if(subjects.begin(),subjects.end(),[temp](Subject::Ptr p){return p->timeSinceLastMeasurement() > temp;}),subjects.end());
    }
  }
}
