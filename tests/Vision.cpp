#include "Vision.h"

Vision::Vision()
{
  m_left_table->PutNumber("ledMode", 1); // Disable lights on boot
  m_right_table->PutNumber("ledMode", 1);
}

int Vision::pose_loop(int i)
{
  /**
   * The main loop for collecting data and determining if it's good
   * Takes and returns an int representing position in data collection
   **/

  if((m_left_table->GetBoolean("tv", false) ||
     m_right_table->GetBoolean("tv", false)) &&
     i <= CONSTANTS::VISION::BUFFER_SIZE)
    {
      Vision::get_raw_data(i);
      i = i++;
    }

  if (i == CONSTANTS::VISION::BUFFER_SIZE &&
      Vision::check_std_dev(Vision::average(m_left_buffer, m_right_buffer)))
    {
      Vision::update_pose(Vision::average(m_left_buffer, m_right_buffer));
      i = 0;
    }
  return i;
}


void Vision::get_raw_data(int i)
{
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     **/
  if (m_left_table->GetBoolean("tv", false))
    {
      m_left_buffer[i] = m_left_table->GetNumberArray("botpose", m_zero_vector);
    }

  if (m_right_table->GetBoolean("tv", false))
    {
      m_right_buffer[i] = m_right_table->GetNumberArray("botpose", m_zero_vector);
    }
}

Vision::Data* Vision::average(Data* buffer_a, Data* buffer_b)
  {

  }

bool Vision::check_std_dev(Data *buffer)
{
  /** Checks if the deviation of any structs in the given array is too large.
   * Returns true if the entire buffer is suitable for use.
   * Returns false if one col is wonk.
   **/
  if (
      Vision::standard_dev(buffer, COL_TYPE::TRANS_X) >= CONSTANTS::VISION::MAX_STD_DEV ||
      Vision::standard_dev(buffer, COL_TYPE::TRANS_Y) >= CONSTANTS::VISION::MAX_STD_DEV ||
      Vision::standard_dev(buffer, COL_TYPE::ROT) >= CONSTANTS::VISION::MAX_STD_DEV
      ) // If any data is bad, throw it all out
    {
      return false;
    }
  else
    {
      return true;
    }
  return false;
}

double Vision::standard_dev(Data *buffer, COL_TYPE col)
{
  /**
   * Calculates the standard deviation of the inputs.
   * Variables are constantly overwritten as values don't need to be kept
   * longer than their used.
  **/

  double mean = 0.0;
  double new_mean = 0.0;
  switch (col)
    {
    case TRANS_X:
      // Declared from heap beacuse it needs to last
      // for a shorter time than it's scope allows.
      mean = 0.0;
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get mean
          mean = mean + buffer[i].trans_x;
        }
      // Get mean cotd.
      mean = mean / (double) CONSTANTS::VISION::BUFFER_SIZE;
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get distance from mean
          buffer[i].trans_x = buffer[i].trans_x - mean;
        }
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Square all values
          buffer[i].trans_x = buffer[i].trans_x * buffer[i].trans_x;
        }
      new_mean = 0;
       for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
         {
           // Take average of all new values
           new_mean = new_mean + buffer[i].trans_x;
         }
       new_mean = new_mean / (double) CONSTANTS::VISION::BUFFER_SIZE;

       // Take sqare root of new mean
       return std::sqrt(new_mean);
       break;

    case TRANS_Y:
      mean = 0.0;
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get mean
          mean = mean + buffer[i].trans_y;
        }
      // Get mean cotd.
      mean = mean / (double) CONSTANTS::VISION::BUFFER_SIZE;
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get distance from mean
          buffer[i].trans_y = buffer[i].trans_y - mean;
        }
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Square all values
          buffer[i].trans_y = buffer[i].trans_y * buffer[i].trans_y;
        }
      new_mean = 0.0;
       for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
         {
           // Take average of all new values
           new_mean = new_mean + buffer[i].trans_y;
         }
       new_mean = new_mean / (double) CONSTANTS::VISION::BUFFER_SIZE;

       // Take sqare root of new mean
       return std::sqrt(new_mean);
       break;

    case ROT:
      mean = 0.0;

      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get mean
          mean = mean + buffer[i].rot_x;
        }
      // Get mean cotd.
      mean = mean / (double) CONSTANTS::VISION::BUFFER_SIZE;
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Get distance from mean
          buffer[i].rot_x = buffer[i].rot_x - mean;
        }
      for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
        {
          // Square all values
          buffer[i].rot_x = buffer[i].rot_x * buffer[i].rot_x;
        }
      new_mean = 0.0;
       for (int i = 0; i <= CONSTANTS::VISION::BUFFER_SIZE; i++)
         {
           // Take average of all new values
           new_mean = new_mean + buffer[i].rot_x;
         }
       new_mean = new_mean / (double) CONSTANTS::VISION::BUFFER_SIZE;

       // Take sqare root of new mean
       return std::sqrt(new_mean);
       break;
    }
  return false;
}



void Vision::update_pose(Data *bot_pose)
{
}


Vision::~Vision()
{
}
