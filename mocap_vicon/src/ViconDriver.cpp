/*
 * Copyright [2015]
 * [Kartik Mohta <kartikmohta@gmail.com>]
 * [Ke Sun <sunke.polyu@gmail.com>]
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

#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <mocap_vicon/ViconDriver.h>

namespace ViconSDK = ViconDataStreamSDK::CPP;

namespace mocap {

  bool ViconDriver::init() {
    std::vector<std::string> batch_list, fixed_batch_list;
    double timeout;
    nh.param("server_address", server_address, std::string("alkaline2"));
    nh.param("model_list", model_list, std::vector<std::string>(0));
    nh.param("batch_list", batch_list, std::vector<std::string>(0));
    nh.param("fixed_batch_list", fixed_batch_list, std::vector<std::string>(0));
    nh.param("batch_timeout", timeout, 1.0);
    nh.param("frame_rate", frame_rate, 100);
    nh.param("max_accel", max_accel, 20.0);
    nh.param("publish_tf", publish_tf, false);
    nh.param("fixed_frame_id", fixed_frame_id, std::string("mocap"));

    frame_interval = 1.0 / static_cast<double>(frame_rate);
    double& dt = frame_interval;
    process_noise.topLeftCorner<6, 6>() =
    0.5*Eigen::Matrix<double, 6, 6>::Identity()*dt*dt*max_accel;
    process_noise.bottomRightCorner<6, 6>() =
    Eigen::Matrix<double, 6, 6>::Identity()*dt*5*max_accel;
    measurement_noise =
    Eigen::Matrix<double, 6, 6>::Identity()*1e-5;
    model_set.insert(model_list.begin(), model_list.end());

    BatchHandler::Ptr batch;
    for(auto it=batch_list.begin();it!=batch_list.end();it++)
    {
      batch = boost::make_shared<BatchHandler>(&nh, (*it), fixed_frame_id, timeout);
      batch->setParameters(process_noise,measurement_noise,frame_rate);
      batches.push_back(batch);
    }
    for(auto it=fixed_batch_list.begin();it!=fixed_batch_list.end();it++)
    {
      batch = boost::make_shared<BatchHandler>(&nh, (*it), fixed_frame_id);
      batch->setParameters(process_noise,measurement_noise,frame_rate);
      batches.push_back(batch);
    }

    timespec ts_sleep;
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 100000000;

    // Connect to the server
    ROS_INFO("Connecting to Vicon Datastream server at %s", server_address.c_str());
    bool is_connected = false;
    for (int retry_cnt = 0; retry_cnt < 10; ++retry_cnt) {
      client->Connect(server_address);
      if(client->IsConnected().Connected) {
        is_connected = true;
        break;
      }
      else
        nanosleep(&ts_sleep, NULL);
    }

    // Report if cannot connect
    if (!is_connected) {
      ROS_WARN("Cannot Connect to Vicon server at %s", server_address.c_str());
      return false;
    }

    // Configure the connection
    ROS_INFO("Successfully Connect to Vicon server at %s", server_address.c_str());
    client->SetStreamMode(ViconSDK::StreamMode::ClientPull);
    client->SetAxisMapping(ViconSDK::Direction::Forward,
                           ViconSDK::Direction::Left, ViconSDK::Direction::Up);
    client->EnableSegmentData();
    if(!client->IsSegmentDataEnabled().Enabled) {
      ROS_WARN("Segment data cannot be enabled.");
      return false;
    }
    ROS_INFO("Successfully configure Vicon server at %s", server_address.c_str());

    // Need to wait for some time after enabling data else you get junk frames
    //struct timespec ts_sleep;
    ts_sleep.tv_sec = 0;
    ts_sleep.tv_nsec = 100000000;
    nanosleep(&ts_sleep, NULL);

    return true;
  }

  void ViconDriver::run() {
    ViconSDK::Result::Enum result = client->GetFrame().Result;
    if (result != ViconSDK::Result::Success)
      return;
    handleFrame();
    return;
  }

  void ViconDriver::disconnect() {
    ROS_INFO_STREAM("Disconnected with the server at "
                    << server_address);
    client->Disconnect();
    return;
  }

  void ViconDriver::handleFrame() {
    int body_count = client->GetSubjectCount().SubjectCount;
    // Assign each subject with a thread
    std::map<std::string, boost::shared_ptr<boost::thread> > subject_threads;

    ViconSDK::Output_GetSegmentGlobalTranslation trans;
    ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat;

    bool in_batch;

    for (int i = 0; i< body_count; ++i) {
      std::string subject_name = client->GetSubjectName(i).SubjectName;
      std::string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;

      // Get the pose for the subject
      trans = client->GetSegmentGlobalTranslation(subject_name, segment_name);
      quat = client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);

      // TODO: Breaks disabling of individual subjects. Fix
      if(trans.Result != ViconSDK::Result::Success ||
         quat.Result != ViconSDK::Result::Success ||
         trans.Occluded || quat.Occluded) {
        continue;
      }

      // Convert the msgs to Eigen type
      Eigen::Quaterniond m_att(quat.Rotation[3], quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
      Eigen::Vector3d m_pos(trans.Translation[0]/1000, trans.Translation[1]/1000, trans.Translation[2]/1000);

      // Feed the new measurement to the subject
      double time = ros::Time::now().toSec();


      // TODO: Publish tf for batches
      in_batch = false;
      for(auto it=batches.begin();it!=batches.end();it++)
      {
        if(subject_name.find((*it)->getName()) != std::string::npos){
          (*it)->addMeasurement(time,m_att,m_pos);
          //in_batch = true;
          break;
        }
      }
      if(in_batch) continue;

      // Process the subject if required
      if (model_set.empty() || model_set.count(subject_name)) {
        // Create a new subject if it does not exist
        if (subjects.find(subject_name) == subjects.end()) {
          subjects[subject_name] = Subject::Ptr(new Subject(&nh, subject_name, fixed_frame_id));
          subjects[subject_name]->setParameters(process_noise, measurement_noise, frame_rate);
        }
        // Handle the subject in a different thread
        subject_threads[subject_name] = boost::shared_ptr<boost::thread>(new boost::thread(&ViconDriver::handleSubject, this, i));

        //handleSubject(i);
      }
    }

    //for(size_t i=0;i<batches.size();i++)
    //for(BatchHandler::Map::iterator it=batches.begin();it != batches.end();it++)
    for(size_t i=0;i<batches.size();i++)
    {
      subject_threads[batches[i]->getName()] = boost::shared_ptr<boost::thread>(new boost::thread(&ViconDriver::handleBatch, this, i));;
    }

    // Wait for all the threads to stop
    for (auto it = subject_threads.begin();
         it != subject_threads.end(); ++it) {
      it->second->join();
    }

    // Send out warnings
    for (auto it = subjects.begin();
         it != subjects.end(); ++it) {
      Subject::Status status = it->second->getStatus();
      if (status == Subject::LOST)
        ROS_WARN("Lose track of subject %s", (it->first).c_str());
      else if (status == Subject::INITIALIZING)
        ROS_WARN("Initialize subject %s", (it->first).c_str());
    }

    return;
  }

  void ViconDriver::handleSubject(const int& sub_idx) {

    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    // We assume each subject has only one segment
    std::string subject_name = client->GetSubjectName(sub_idx).SubjectName;
    std::string segment_name = client->GetSegmentName(subject_name, 0).SegmentName;
    // Get the pose for the subject
    ViconSDK::Output_GetSegmentGlobalTranslation trans =
    client->GetSegmentGlobalTranslation(subject_name, segment_name);
    ViconSDK::Output_GetSegmentGlobalRotationQuaternion quat =
    client->GetSegmentGlobalRotationQuaternion(subject_name, segment_name);
    write_lock.unlock();

    //boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    if(trans.Result != ViconSDK::Result::Success ||
       quat.Result != ViconSDK::Result::Success ||
       trans.Occluded || quat.Occluded) {
      subjects[subject_name]->disable();
      return;
    }

    // Convert the msgs to Eigen type
    Eigen::Quaterniond m_att(quat.Rotation[3],
                             quat.Rotation[0], quat.Rotation[1], quat.Rotation[2]);
    Eigen::Vector3d m_pos(trans.Translation[0]/1000,
                          trans.Translation[1]/1000, trans.Translation[2]/1000);

    // Re-enable the object if it is lost previously
    if (subjects[subject_name]->getStatus() == Subject::LOST) {
      subjects[subject_name]->enable();
    }

    // Feed the new measurement to the subject
    double time = ros::Time::now().toSec();
    subjects[subject_name]->processNewMeasurement(time, m_att, m_pos);
    //read_lock.unlock();

    // Publish tf if requred
    if (publish_tf &&
        subjects[subject_name]->getStatus() == Subject::TRACKED) {

      Eigen::Quaterniond att = subjects[subject_name]->getAttitude();
      Eigen::Vector3d pos = subjects[subject_name]->getPosition();
      tf::Quaternion att_tf;
      tf::Vector3 pos_tf;
      tf::quaternionEigenToTF(att, att_tf);
      tf::vectorEigenToTF(pos, pos_tf);

      tf::StampedTransform stamped_transform =
      tf::StampedTransform(tf::Transform(att_tf, pos_tf),
                           ros::Time::now(), fixed_frame_id, subject_name + "/base");
      write_lock.lock();
      tf_publisher.sendTransform(stamped_transform);
      write_lock.unlock();
    }

    return;
  }

  void ViconDriver::handleBatch(const int& sub_idx)
  {
    boost::unique_lock<boost::shared_mutex> write_lock(mtx);
    write_lock.unlock();

    batches[sub_idx]->processMeasurements();

    // Publish tf if requred
    if (publish_tf) {
      Eigen::Quaterniond att;
      Eigen::Vector3d pos;
      tf::Quaternion att_tf;
      tf::Vector3 pos_tf;
      tf::StampedTransform stamped_transform;
      int N = batches[sub_idx]->getNumSubjects();
      for(int i=0;i<N;i++){
        att = batches[sub_idx]->getAttitude(i);
        pos = batches[sub_idx]->getPosition(i);
        tf::quaternionEigenToTF(att, att_tf);
        tf::vectorEigenToTF(pos, pos_tf);

        stamped_transform = tf::StampedTransform(tf::Transform(att_tf, pos_tf),
                             ros::Time::now(), fixed_frame_id, batches[sub_idx]->getSubjectName(i)+"/base");
        write_lock.lock();
        tf_publisher.sendTransform(stamped_transform);
        write_lock.unlock();
      }
    }
  }
}
