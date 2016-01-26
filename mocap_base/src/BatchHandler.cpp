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

namespace hungarian{
  // Hungarian algorithm implementation from: https://github.com/emmarose88/HungarianAlgorithm
  const int MATCH_MIN = 0;
  const int MATCH_MAX = 1;
  /*
   * reduce
   * reduces matrix based on row and column minimums
   */
  void reduce(Eigen::MatrixXd& m) {
    // subtract row minimum from each row
    for (int i=0; i<m.rows(); i++) {
      float minElement = m.row(i).minCoeff();
      Eigen::VectorXd rMinusMin(m.rows());
      rMinusMin.fill(-minElement);
      m.row(i) += rMinusMin;
    }
  }

  /*
   * hasMark
   * if there is a starred/primed zero in the given row/col, returns it's index
   * else, returns -1
   */
  int hasMark(Eigen::VectorXd& v) {
    for (int i=0; i<v.size(); i++) {
      if (v(i)) {
        return i;
      }
    }
    return -1;
  }

  /*
   * swapStarsAndPrimes
   * Swap stars and primes based on step 5 of Hungarian algorithm
   * Z0 is uncovered primed zero we've found
   * Z1 is the stared zero in the column of Z0 (if any)
   * Z2 is the primed zero in the row of Z1 (will always be one)
   * ...continue series until we reach a primed zero with no starred zero in its column
   * Unstar each starred zero, star each primed zero, erase all primes and uncover every line in the matrix
   */
  void swapStarsAndPrimes(int i, int j, Eigen::MatrixXd& stars, Eigen::MatrixXd& primes) {
    int primeRow = i;
    int primeCol = j;

    bool done = false;
    while (!done) {
      // find row index of row that has a 0* in the same col as the current 0'
      Eigen::VectorXd col = stars.col(primeCol);
      int starInPrimeColRow = hasMark(col);

      if (starInPrimeColRow < 0) {
        // star the prime we're looking at
        primes(primeRow, primeCol) = 0;
        stars(primeRow, primeCol) = 1;
        done = true;
      }
      else {
        // find which col has a 0' in the same row as z1
        Eigen::VectorXd row = primes.row(starInPrimeColRow);
        int primeInStarRowCol = hasMark(row);

        // star first primed zero
        primes(primeRow, primeCol) = 0;
        stars(primeRow, primeCol) = 1;
        //primes(starInPrimeColRow, primeInStarRowCol) = 0;
        //stars(starInPrimeColRow, primeInStarRowCol) = 1;

        // unstar starred zero
        stars(starInPrimeColRow, primeCol) = 0;

        // set index of last prime, will check it's column for 0*s next
        primeRow = starInPrimeColRow;
        primeCol = primeInStarRowCol;
      }
    }
    // clear primes
    primes.fill(0);
  }

  /*
   * findMatching
   * implementation of the Hungarian matching algorithm
   * referenced from: http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
   */
  void findMatching(Eigen::MatrixXd& m, Eigen::MatrixXd& result, int type) {
    Eigen::MatrixXd n = m; // make a copy of m for reducing
    int dim = n.rows(); // dimension of matrix, used for checking if we've reduced
    // the matrix enough yet

    Eigen::MatrixXd stars(m.rows(), m.cols()); // matrix for storing our "starred" 0s (0*)
    stars.fill(0);
    Eigen::MatrixXd primes(m.rows(), m.cols()); // matrix for storing our "primed" 0s (0')
    primes.fill(0);
    Eigen::VectorXd rowCover(m.rows()); // keep track of which rows are "covered"
    rowCover.fill(0);
    Eigen::VectorXd colCover(m.cols()); // keep track of which columns are "covered"
    colCover.fill(0);

    // to do maximization rather than minimization, we have to
    // transform the matrix by subtracting every value from the maximum
    if (type == MATCH_MAX) {
      float max = n.maxCoeff();
      Eigen::MatrixXd maxMat(n.rows(), n.cols());
      maxMat.fill(max);
      n = maxMat - n;
    }

    // Step 1
    // Reduce matrix
    reduce(n);

    // Step 2
    // Find a zero in the matrix. If there is no starred zero in
    // its row or column, star Z. Repeat for each element in the matrix.
    for (int i=0; i<n.rows(); i++) {
      for (int j=0; j<n.cols(); j++) {
        if (n(i,j) == 0 && !rowCover(i) && !colCover(j)) {
          stars(i,j) = 1;
          rowCover(i) = 1;
          colCover(j) = 1;
        }
      }
    }
    // covers need to be cleared for following steps
    rowCover.fill(0);
    colCover.fill(0);

    while (true) {
      // Step 3
      // Cover all columns that have a starred zero
      // If the number of columns with starred zeroes equals the matrix
      // dimensions, we are done! Otherwise, move on to step 4.
step3:
      for (int j=0; j<n.cols(); j++) {
        Eigen::VectorXd col = stars.col(j);
        if (hasMark(col) >= 0) {
          colCover(j) = 1;
        }
      }
      if (colCover.sum() == dim) {
        result = stars;
        return;
      }

      // Step 4
      // Find a non-covered zero and prime it
step4:
      for (int i=0; i<n.rows(); i++) {
        for (int j=0; j<n.cols(); j++) {
          if (n(i,j) == 0 && !rowCover(i) && !colCover(j)) {
            primes(i,j) = 1;
            // if no starred zero in the row...
            Eigen::VectorXd row = stars.row(i);
            if (hasMark(row) < 0) {
              // Step 5
              // swap stars and primes
              swapStarsAndPrimes(i, j, stars, primes);

              // clear lines
              rowCover.fill(0);
              colCover.fill(0);

              goto step3;
            }
            else {
              // cover row
              rowCover(i) = 1;

              // uncover column of the starred zero in the same row
              int col = hasMark(row);
              colCover(col) = 0;
            }
          }
        }
      }

      // Step 6
      // Should now be no more uncovered zeroes
      // Get the minimum uncovered element
      float min = 1000000;
      for (int i=0; i<n.rows(); i++) {
        for (int j=0; j<n.cols(); j++) {
          if (!rowCover(i) && !colCover(j) && n(i,j) < min) {
            min = n(i,j);
          }
        }
      }

      // Subtract minimum from uncovered elements, add it to elements covered twice
      for (int i=0; i<n.rows(); i++) {
        for (int j=0; j<n.cols(); j++) {
          if (!rowCover(i) && !colCover(j)) {
            n(i,j) -= min;
          }
          else if (rowCover(i) && colCover(j)) {
            n(i,j) += min;
          }
        }
      }

      goto step4;
    }
  }
}

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

  // Get number of active objects
  int BatchHandler::getNumSubjects()
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects.size();
  }

  // Get object state for given index
  const Eigen::Quaterniond& BatchHandler::getAttitude(const int& idx)
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects[idx]->getAttitude();
  }
  const Eigen::Vector3d& BatchHandler::getPosition(const int& idx)
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects[idx]->getPosition();
  }
  const Eigen::Vector3d& BatchHandler::getAngularVel(const int& idx)
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects[idx]->getAngularVel();
  }
  const Eigen::Vector3d& BatchHandler::getLinearVel(const int& idx)
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects[idx]->getLinearVel();
  }
  const std::string& BatchHandler::getSubjectName(const int& idx)
  {
    boost::shared_lock<boost::shared_mutex> read_lock(mtx);
    return subjects[idx]->getName();
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
    std::string object_name;
    if(subjects.size() < measurements.size())
    {
      object_name = name+std::to_string(object_count);
      printf("BatchHandler.cpp: Adding object '%s' to batch.\n\r", object_name.c_str());
      s = boost::make_shared<Subject>(nh_ptr,object_name+"/batch",parent_frame);
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
    // N_m should be greater than N_s, as per addMeasurement (which adds objects if measurements is greater)
    Eigen::MatrixXd m(N_s,N_s);
    Eigen::Vector3d pos;
    m.fill(0);
    double max_val = 0;
    for(size_t i=0;i<N_s;i++){
      for(size_t j=0;j<N_m;j++){
        subjects[i]->getPredictedPosition(measurements[j]->time,pos);
        m(i,j) = (pos - measurements[j]->position).squaredNorm();
        if(m(i,j) > max_val)
          max_val = m(i,j);
      }
    }
    if(N_s > N_m)
      m.rightCols(N_s-N_m).fill(max_val);
    Eigen::MatrixXd result(N_s,N_s);
    result.fill(0);

    hungarian::findMatching(m,result,hungarian::MATCH_MIN);
    for(size_t i=0;i<N_s;i++)
    {
      for(size_t j=0;j<N_m;j++){
        if(result(i,j) == 1){
          subjects[i]->processNewMeasurement(measurements[j]->time,measurements[j]->attitude,measurements[j]->position);
          break;
        }
      }
    }
    //for(size_t i=0;i<N_s && i<N_m;i++)
    //{
    //  subjects[i]->processNewMeasurement(measurements[i]->time,measurements[i]->attitude,measurements[i]->position);
    //}
    measurements.clear();

    double temp = measurement_timeout;
    // Remove inactive objects if batch is not fixed
    if(measurement_timeout > 0){
      subjects.erase(std::remove_if(subjects.begin(),subjects.end(),[temp](Subject::Ptr p){return p->timeSinceLastMeasurement() > temp;}),subjects.end());
    }
  }
}
