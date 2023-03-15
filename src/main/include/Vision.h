#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <vector>
#include <units/angle.h>
#include <units/length.h>
#include <cmath>
#include "Constants.h"
//#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "wpi/span.h"

#include "Drivetrain.hpp"
#include "Odometry.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <numeric>

class Vision
{
  public:
  Vision(/* args */);
  ~Vision();

  int test = 10;
  struct Data
  {
    double trans_x;
    double trans_y;
    double rot_x;
    bool is_good; // Determine if data was written

    Data(){};
    Data(std::vector<double> in_vec)
    {
      if (in_vec.size() >= 6)
      {
        trans_x = in_vec[0];
        trans_y = in_vec[1];
        rot_x = in_vec[5];
      } 
    };
  };


  void get_raw_data(int i);
  bool pose_loop();

  double standard_dev(std::vector<double> v);
private:

  /// @brief Tables for data to be stored in.
  std::vector<Vision::Data> m_left_buffer{CONSTANTS::VISION::BUFFER_SIZE};
  std::vector<Vision::Data> m_right_buffer{CONSTANTS::VISION::BUFFER_SIZE};

  Data nonsense;

  /// @brief Where in the circular buffer the program is operating on
  int m_index_pt = 0;
  std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

  std::shared_ptr<nt::NetworkTable> m_left_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");

  std::shared_ptr<nt::NetworkTable> m_right_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");

  /**
   * @brief Converts from a table of data to a vector
   * @param f the table value to be collected (such as trans_x)
   * @param v the table to be operated on
   * 
   * @return a vector of only the values defined in f. 
   * Will filter out bad data.
  */
  std::vector<double> collect(double Data::* f, std::vector<Data> const& v);

/**
 * @brief updates the odometry for the robot
 * @param bot_pose the values to update to
*/
  void update_pose(Data bot_pose);

  /**
   * @brief checks to see if the standard deviation is within a known threshold
   * @param buffer the data to be operated on
   * @return True if within threshold. False if not.
  */
  bool check_std_dev(std::vector<Data> buffer);

/**
 * @brief A function to determine how many elements in the vector were taken 
 * from the apriltags as opposed to filler data or nulls.
 * 
 * @param data the dataset to check
 * @return the number of elements that were taken from apriltags.
*/
  int find_good_frames(std::vector<Data> data);

/**
 * @brief finds the average of two vectors.
 * @param a first vector to average
 * @param b second vector to average
 * @return the average vector
*/
  double average(std::vector<double> buffer_a, std::vector<double> buffer_b);

  /**
   * @brief takes the average of one vector. Exactly what is says on the tin.
  */
  double average(std::vector<double> v);
};