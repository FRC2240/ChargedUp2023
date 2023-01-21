#include "Vision.h"

Vision::Vision()
{
  m_alpha_table->PutNumber("ledMode", 1); // Disable lights on boot
  m_beta_table->PutNumber("ledMode", 1);
}


void Vision::update_pose(std::vector<double> bot_pose)
{
  // Note: 3d data is dropped
  units::meter_t trans_x{bot_pose[0]};
  units::meter_t trans_y{bot_pose[1]};
  frc::Rotation2d rot{units::degree_t(bot_pose[3])};
  // units::meter_t x, y, Rotation::2d theta
  frc::Pose2d pose{trans_x, trans_y, rot};
  Odometry::reset_position_from_vision(pose);

}

double Vision::standard_dev(double a,
                            double b,
                            double c,
                            double d,
                            double e)
{
  /**
   * Calculates the standard deviation of the inputs.
   * Variables are constantly overwritten as values don't need to be kept
   * longer than their used.
  **/
  // Calculate distance from mean
  double main_op = (a + b + c + d + e) /5;
  a = a - main_op;
  b = b - main_op;
  c = c - main_op;
  d = d - main_op;
  e = e - main_op;
 // Square all values
  a = a * a;
  b = b * b;
  c = c * c;
  d = d * d;
  e = e * e;

  // Take average of inputs
  main_op = a + b + c + d + e;
  main_op = main_op / 5;

  // take root of new mean
  return std::sqrt(main_op);
}

std::vector<double> Vision::two_vector_avg(
                                           std::vector<double> a,
                                           std::vector<double> b
                                           )
  {
    std::vector<double> avg;

    for (int i = 0; i < 5; i++)
      {
        avg[i] = a[i] + b[i];
        avg[i] = avg[i] / 2;
      }
    return avg;

  }
std::vector<double> Vision::five_vector_avg(
                                            std::vector<double> a,
                                            std::vector<double> b,
                                            std::vector<double> c,
                                            std::vector<double> d,
                                            std::vector<double> e
                                            )
  {
    std::vector<double> avg;

    for (int i = 0; i < 5; i++)
      {
        avg[i] = a[i] + b[i] + c[i] + d[i] + e[i];
        avg[i] = avg[i] / 5;
      }
    return avg;
  }

std::vector<double> Vision::get_raw_data()
  {
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     *Returns a vector of doubles
     **/

    std::vector<double> bot_pose;

    if (m_alpha_table->GetBoolean("tv", false) &&
        m_beta_table->GetBoolean("tv", false))
      {
        return two_vector_avg(
                              m_alpha_table->GetNumberArray("botpose", m_zero_vector),
                              m_beta_table->GetNumberArray("botpose", m_zero_vector)
                              );
      }
    else
      {
        if (m_alpha_table->GetBoolean("tv", false))
          {
            return m_alpha_table->GetNumberArray("botpose", m_zero_vector);
          }
        else if (m_beta_table->GetBoolean("tv", false))
          {
            return m_beta_table->GetNumberArray("botpose", m_zero_vector);
          }
      }
    return m_zero_vector;
  }

int Vision::pose_loop(int i /* = 0*/)
{
  if(m_alpha_table->GetBoolean("tv", false) &&
     m_beta_table->GetBoolean("tv", false ))
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
          double total_std_dev = 0;
          for (int std_dev_i = 0; std_dev_i < 5; std_dev_i++)
            {
              total_std_dev = total_std_dev +
                Vision::standard_dev(
                                     m_result_0[std_dev_i],
                                     m_result_1[std_dev_i],
                                     m_result_2[std_dev_i],
                                     m_result_3[std_dev_i],
                                     m_result_4[std_dev_i]);
            }

          if (total_std_dev < CONSTANTS::VISION::MAX_STD_DEV)
            {
              Vision::update_pose(
              Vision::five_vector_avg(
                                      m_result_0,
                                      m_result_1,
                                      m_result_2,
                                      m_result_3,
                                      m_result_4
                                      ));
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

Vision::~Vision(){}
