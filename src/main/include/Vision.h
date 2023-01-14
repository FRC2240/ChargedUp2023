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

class Vision
{
private:
  std::vector<double> m_zero_vector = {0,0,0,0,0,0};
  //QUESTION: is it a better idea to set this to zero or null?

  std::shared_ptr<nt::NetworkTable> m_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-brute");

  void update_pose(std::vector<double> bot_pose);

  double standard_dev( double a,
                       double b,
                       double c,
                       double d,
                       double e
                       );

  std::vector<double> m_result_0;
  std::vector<double> m_result_1;
  std::vector<double> m_result_2;
  std::vector<double> m_result_3;
  std::vector<double> m_result_4;

  std::vector<double> five_vector_avg(
                                      std::vector<double> a,
                                      std::vector<double> b,
                                      std::vector<double> c,
                                      std::vector<double> d,
                                      std::vector<double> e
                                      );

public:
  Vision(/* args */);
  ~Vision();


  std::vector<double> get_raw_data();
  int pose_loop(int i = 0);
  std::vector<double> get_xy_offset();
  units::degrees get_rotation_by_tag();

};
#endif
