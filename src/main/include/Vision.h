#ifndef VISION_H_
#define VISION_H_

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Rotation2d.h>
#include <vector>
#include <units/angle.h>
#include <units/length.h>
#include "Constants.h"
//#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
//#include "wpi/span.h"

class Vision
{
private:

std::shared_ptr<nt::NetworkTable> m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight-brute");
public:
Vision(/* args */);
~Vision();


/*frc::Pose2d */ double get_field_pos_by_tag();
std::vector<double> get_xy_offset();
units::degrees get_rotation_by_tag();

};
#endif
