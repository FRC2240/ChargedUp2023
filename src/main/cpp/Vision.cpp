#include "Vision.h"

Vision::Vision()
{
  m_table->PutNumber("ledMode", 1); // Disable lights on boot
}


/*frc::Pose2d */ double Vision::get_field_pos_by_tag()
  {
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     *Return a Pose2d object
     **/
    //if (m_table>GetNumber("tv",0.0))
      //{
        return m_table->GetNumber("botpose",0.0);

      //}
  }


std::vector<double> get_xy_offset()
  {
    /*
     * Gets the offset of the robot from the taget
     * Just use the built in tools
     * https://docs.limelightvision.io/en/latest/apriltags_in_3d.html
     *
     */

  }

units::degrees get_rotation_by_tag()
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