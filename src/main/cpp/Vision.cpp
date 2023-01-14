#include "Vision.h"

Vision::Vision()
{
  m_table->PutNumber("ledMode", 1); // Disable lights on boot
}


void Vision::update_pose(std::vector<double> bot_pose)
{
    units::meter_t trans_x(bot_pose[0]);
    units::meter_t trans_y(bot_pose[1]);
    units::meter_t trans_z(bot_pose[2]);
    //Rotation is X,Y,Z, I don't know why we need this much data.
    units::meter_t rot_x(bot_pose[3]);
    units::meter_t rot_y(bot_pose[4]);
    units::meter_t rot_z(bot_pose[5]);
    // TODO: update pose
}

double Vision::standard_dev(std::vector<double> a,
                            std::vector<double> b,
                            std::vector<double> c,
                            std::vector<double> d,
                            std::vector<double> e)
{
  // Calculates the standard deviation of the inputs
  //
}

std::vector<double> Vision::get_raw_data()
  {
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     *Returns a vector of doubles
     **/
    std::vector<double> bot_pose = m_table->GetNumberArray("botpose", m_zero_vector);

    return bot_pose;
  }

int Vision::pose_loop(int i /* = 0*/)
{
  if(m_table->GetBoolean("tv", false))
    {

      switch (i)
        {
        case 0:
          m_result_0 = Vision::get_raw_data();
          return i++;
          break;

        case 1:
          m_result_1 = Vision::get_raw_data();
          return i++;
          break;

        case 2:
          m_result_2 = Vision::get_raw_data();
          return i++;
          break;

        case 3:
          m_result_3 = Vision::get_raw_data();
          return i++;
          break;

        case 4:
          m_result_4 = Vision::get_raw_data();
          return i++;
          break;

        case 5:
          if (Vision::standard_dev(m_result_0, m_result_1, m_result_2, m_result_3, m_result_4) < CONSTANTS::VISION::MAX_STD_DEV)
            {
              // Update Pose
              return 0;
            }
          else
            {
              return 0;
            }
          break;

        }

    }

  return i;

}

std::vector<double> Vision::get_xy_offset()
{
  /*
   * Gets the offset of the robot from the taget
   * Just use the built in tools
   * https://docs.limelightvision.io/en/latest/apriltags_in_3d.html
   *
   */

}

units::degrees Vision::get_rotation_by_tag()
{
  /* Gets the orientation of the robot based on the tag
     * Use the proper tag and not the offset one
     **/
  if (nt::NetworkTableInstance::GetDefault()
      .GetTable("limelight")->GetNumber("tv", 0.0))
    {
      //        const units::degree rot{nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ts", 0.0)};

      //      return rot;
    }
  //There is not return here on purpose.
  //That way pose won't be fed garbage data.
}

Vision::~Vision(){}
