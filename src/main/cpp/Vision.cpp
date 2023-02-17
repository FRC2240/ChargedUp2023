#include "Vision.h"

Vision::Vision()
{
  m_left_table->PutNumber("ledMode", 1); // Disable lights on boot
  m_right_table->PutNumber("ledMode", 1);
}

void Vision::pose_loop()
{
  /**
   * The main loop for collecting data.
   * OVERALL STRUCTURE:
   * 1. Get data and save it to buffer (repeat BUFFER_SIZE times)
   * 2. Check std dev (every cycle)
   * 3. If valid, update pose
   **/

  if constexpr (CONSTANTS::DEBUGGING)
    {

      if (m_index_pt == CONSTANTS::VISION::BUFFER_SIZE)
        {
        }
    }
  if ((m_right_table->GetNumber("tv", 0.0) == 1.0) || (m_left_table->GetNumber("tv", 0.0) == 1.0))
    {
      Vision::get_raw_data(m_index_pt);

        // Don't move forward if there is no data
          m_index_pt++;
    }

  if (m_index_pt >= CONSTANTS::VISION::BUFFER_SIZE)
    {
      m_index_pt = 0;
    }

  bool dev_check_l = Vision::check_std_dev(m_left_buffer);
  bool dev_check_r = Vision::check_std_dev(m_right_buffer);

  Data pose_ret_val;
  if (dev_check_l && dev_check_r)
    {
      pose_ret_val.trans_x = Vision::average(
                                             Vision::collect(&Data::trans_x,
                                                             m_left_buffer),
                                             Vision::collect(&Data::trans_x,
                                                             m_right_buffer));
          pose_ret_val.trans_y = Vision::average(
                                             Vision::collect(&Data::trans_y,
                                                             m_left_buffer),
                                             Vision::collect(&Data::trans_y,
                                                             m_right_buffer));
          pose_ret_val.rot_x = Vision::average(
                                           Vision::collect(&Data::rot_x,
                                                           m_left_buffer),
                                           Vision::collect(&Data::rot_x,
                                                           m_right_buffer));
          Vision::update_pose(pose_ret_val);
    }
  else if (dev_check_l)
    {
      pose_ret_val.trans_x = Vision::average(Vision::collect(&Data::trans_x, m_left_buffer));
      pose_ret_val.trans_y = Vision::average(Vision::collect(&Data::trans_y, m_left_buffer));
      pose_ret_val.rot_x = Vision::average(Vision::collect(&Data::rot_x, m_left_buffer));
     Vision::update_pose(pose_ret_val);
    }
  else if (dev_check_r)
    {
      pose_ret_val.trans_x = Vision::average(Vision::collect(&Data::trans_x, m_right_buffer));
      pose_ret_val.trans_y= Vision::average(Vision::collect(&Data::trans_y, m_right_buffer));
      pose_ret_val.rot_x = Vision::average(Vision::collect(&Data::rot_x, m_right_buffer));
      Vision::update_pose(pose_ret_val);
    }

}

std::vector<double> Vision::collect(double Data::* f, std::vector<Data> const& v)
{
 /** Takes a data array and makes it a vector.
  * Ask Erik for more information.
  * No touchy
  **/
  std::vector<double> output;
  for (auto const& elem : v)
    {
      if (elem.is_good)
        {
          output.push_back(elem.*f);
        }
    }
  return output;
}



void Vision::get_raw_data(int i)
{
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     **/
  if (m_left_table->GetNumber("tv", 0.0))
    {
      m_left_buffer[i] = m_left_table->GetNumberArray("botpose", m_zero_vector);
      m_left_buffer[i].is_good = true;
    }
  else
    {
      m_left_buffer[i].is_good = false;
    }

  if (m_right_table->GetNumber("tv", 0.0))
    {
      m_right_buffer[i] = m_right_table->GetNumberArray("botpose", m_zero_vector);
      m_right_buffer[i].is_good = true;
    }
  else
    {
      m_right_buffer[i].is_good = false;
    }
}

double Vision::average(std::vector<double> v)
{
  double mean = 0.0;

  for (auto value : v)
    {
      mean += value;
    }
 return (mean / (double) v.size());
}

double Vision::average(std::vector<double> buffer_a,
                       std::vector<double> buffer_b)
{
  std::vector<double> v;

  #warning "MINOR: You're doing this wrong. Join buffers better"
  for (unsigned short int i = 0; i < buffer_a.size(); i++)
    {
      v.push_back(buffer_a[i]);
    }

   for (unsigned short int i = 0; i < buffer_b.size(); i++)
    {
      v.push_back(buffer_b[i]);
    }

  // https://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  double mean = 0.0;

  for (auto value : v)
    {
      mean += value;
    }
 return (mean / (double) v.size());

}

bool Vision::check_std_dev(std::vector<Data> buffer)
{
  /** Checks if the deviation of any structs in the given array is too large.
   * Returns true if the entire buffer is suitable for use.
   * Returns false if one col is wonk.
   **/
  if (Vision::find_good_frames(buffer) >=
      CONSTANTS::VISION::MIN_GOOD_FRAMES)
    {
      double x = Vision::standard_dev(Vision::collect(&Data::trans_x, buffer));
      double y = Vision::standard_dev(Vision::collect(&Data::trans_y, buffer));
      double z = Vision::standard_dev(Vision::collect(&Data::rot_x, buffer));
      if (
          ((x >= CONSTANTS::VISION::MIN_STD_DEV) &&
           (x <= CONSTANTS::VISION::MAX_STD_DEV)) ||

          ((y >= CONSTANTS::VISION::MIN_STD_DEV) &&
           (y <= CONSTANTS::VISION::MAX_STD_DEV)) ||

          ((z >= CONSTANTS::VISION::MIN_STD_DEV_ROT) &&
           (z <= CONSTANTS::VISION::MAX_STD_DEV_ROT))
          )
        {
         return true;
        }

    }
  return false;
}


int Vision::find_good_frames(std::vector<Data> data)
{
  int frames = 0;
  for (int i = 0; i < CONSTANTS::VISION::BUFFER_SIZE; i++)
    {
      if (data[i].is_good)
        {
          frames++;
        }
    }
  return frames;
} 

double Vision::standard_dev(std::vector<double> v)
{
//https://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos
double sum = std::accumulate(v.begin(), v.end(), 0.0);
double mean = sum / v.size();

double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
return stdev;
}

void Vision::update_pose(Data bot_pose)
{
  // Note: 3d data is dropped
  units::meter_t trans_x{bot_pose.trans_x};
  units::meter_t trans_y{bot_pose.trans_y};
  frc::Rotation2d rot{units::degree_t(bot_pose.rot_x)};
  // units::meter_t x, y, Rotation::2d theta
  frc::Pose2d pose{trans_x, trans_y, rot};
  Odometry::reset_position_from_vision(pose);
}


Vision::~Vision()
{
}
