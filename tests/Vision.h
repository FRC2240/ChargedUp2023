#ifndef VISION_H_
#define VISION_H_

#include <vector>
#include <cmath>
#include "Constants.h"


class Vision
{
  public:
  Vision(/* args */);
  ~Vision();

  enum MEASURE_TYPE {ROTATION, TRANSLATION};
  enum COL_TYPE {TRANS_X, TRANS_Y, ROT}; //BLAHAJ
  enum POSITION {LEFT, RIGHT};

  struct Data
  {
    double trans_x;
    double trans_y;
    double rot_x;

    Data(){};
    Data(std::vector<double> in_vec)
    {
      trans_x = in_vec[0];
      trans_y = in_vec[1];
      rot_x = in_vec[3];

    };
  };


  void get_raw_data(int i);
  int pose_loop(int i = 0);

private:

  Data m_left_buffer[CONSTANTS::VISION::BUFFER_SIZE];
  Data m_right_buffer[CONSTANTS::VISION::BUFFER_SIZE];


  std::vector<double> m_zero_vector = {0,0,0,0,0,0};
  //QUESTION: is it a better idea to set this to zero or null?

  std::shared_ptr<nt::NetworkTable> m_left_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");

  std::shared_ptr<nt::NetworkTable> m_right_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");


  void update_pose(Data* bot_pose);

  bool check_std_dev(Data* buffer);

  double standard_dev(Data* buffer, COL_TYPE col);

  Data* average(Data* buffer_a, Data* buffer_b);

};
#endif
