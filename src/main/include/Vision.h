#ifndef VISION_H_
#define VISION_H_

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
  void pose_loop();

  // TODO: make private
  std::vector<Vision::Data> m_left_buffer{CONSTANTS::VISION::BUFFER_SIZE};

  std::vector<Vision::Data> m_right_buffer{CONSTANTS::VISION::BUFFER_SIZE};

  double standard_dev(std::vector<double> v);
private:
  int m_index_pt = 0;
  std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

  std::shared_ptr<nt::NetworkTable> m_left_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");

  std::shared_ptr<nt::NetworkTable> m_right_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");

  std::vector<double> collect(double Data::* f, std::vector<Data> const& v);

  void update_pose(Data bot_pose);

  bool check_std_dev(std::vector<Data> buffer);

  int find_good_frames(std::vector<Data> data);

  double average(std::vector<double> buffer_a, std::vector<double> buffer_b);
  double average(std::vector<double> v);


};
#endif
