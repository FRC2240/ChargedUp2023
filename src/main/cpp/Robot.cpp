#include "Robot.hpp"

/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

static auto rotation_joystick = false;

static frc::SendableChooser<std::function<void()>> traj_selector;

static auto field_centric = true;

/******************************************************************/
/*                        Public Variables                        */
/******************************************************************/

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

// Needed to flash on/off temp warnings on SmartDashboard/ShuffleBoard
// static bool drivers_flashing_red = false;
// static bool turners_flashing_red = false;

Robot::Robot()
{
  // setup RobotStates
  RobotState::IsEnabled = [this]()
  { return IsEnabled(); };

  RobotState::IsDisabled = [this]()
  { return IsDisabled(); };

  RobotState::IsAutonomous = [this]()
  { return IsAutonomous(); };

  RobotState::IsAutonomousEnabled = [this]()
  { return IsAutonomousEnabled(); };

  RobotState::IsTeleop = [this]()
  { return IsTeleop(); };

  RobotState::IsTeleopEnabled = [this]()
  { return IsTeleopEnabled(); };

  RobotState::IsTest = [this]()
  { return IsTest(); };

  // Call the inits for all subsystems here
  Drivetrain::init();
  std::cout << "Drivtrain started \n";
  Odometry::putField2d();
  std::cout << "Odometry putfield done \n";

  /* legacy
  frc::SmartDashboard::PutData("Traj Selector", &traj_selector);
  frc::SmartDashboard::PutBoolean("Traj Reversed", Trajectory::reverse_trajectory);
  */

  // Auto paths
  m_chooser.AddOption(Robot::CIRCLE, Robot::CIRCLE);
  m_chooser.AddOption(Robot::LINE, Robot::LINE);
  m_chooser.AddOption(Robot::NON_HOLONOMIC, Robot::NON_HOLONOMIC);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  std::cout << "robot object created \n";
  std::cout << "go get em tiger" << std::endl;
}

void Robot::RobotInit()
{
  Odometry::putField2d();
  std::cout << "RobotInit done \n";

  m_grippad.retract();
}

void buttonManager()
{
  if (BUTTON::DRIVETRAIN::FIELD_CENTRIC())
  {
    std::cout << field_centric
              << "\n";
    field_centric = !field_centric;
  }
}

void tankDrive() {}

void tunePID() {}

void tuneFF() {}

void swerveDrive(bool const &field_relative)
{
  auto const left_right = -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND) *
                          Drivetrain::TELEOP_MAX_SPEED;

  auto const front_back = -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND) *
                          Drivetrain::TELEOP_MAX_SPEED;
  // std::cout << "left_right = " << BUTTON::PS5.GetX() << "front_back = " << BUTTON::PS5.GetY() << std::endl;

  if (rotation_joystick)
  {
    // Multiplied by 10 to avoid rounding to 0 by the atan2() method
    double const rotate_joy_x = BUTTON::DRIVETRAIN::RX() * 10;
    double const rotate_joy_y = -BUTTON::DRIVETRAIN::RY() * 10;

    // If we aren't actually pressing the joystick, leave rotation at previous
    if (std::abs(rotate_joy_x) > 0.1 || std::abs(rotate_joy_y) > 0.1)
    {
      // Get degree using arctan, then convert from unit circle to front-centered values with positive being CW
      Drivetrain::faceDirection(front_back, left_right, -units::radian_t{atan2(rotate_joy_y, rotate_joy_x)} + 90_deg, field_relative);
    }
    else
      Drivetrain::drive(front_back, -left_right, units::radians_per_second_t{0}, field_relative);
  }
  else
  {
    auto const rot = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, left_right, rot, field_relative);
  }
}

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
void Robot::RobotPeriodic()
{
  Trajectory::reverse_trajectory = frc::SmartDashboard::GetBoolean("Traj Reversed", Trajectory::reverse_trajectory);
  // std::cout << "Robot Periodic \n";

  // if (m_arm.position > m_arm.ARM_FLARE_LOW && m_arm.position < m_arm.ARM_FLARE_HIGH){
  //   m_wrist.Follow_Flare(m_arm.position);
  // }
  // else {
  // std::cout << m_arm.position << std::endl;
  m_wrist.Follow(m_arm.position);
  // }
  m_arm.Read_Position();
}

void Robot::AutonomousInit()
{

  m_grippad.retract();

  if (m_autoSelected == TEST)
  {
    m_autoSequence = &m_testSequence;
  }

  m_autoAction = m_autoSequence->front();

  // Start aiming

  // m_autoSelected = m_chooser.GetSelected();
  std::cout << "auto selected \n";

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  if (m_autoSelected == CIRCLE)
  {
    deployDirectory = "Circle";
  }

  if (m_autoSelected == LINE)
  {
    deployDirectory = "30 degree turn";
  }

  if (m_autoSelected == LINE)
  {
    deployDirectory = "Straight Line";
  }

  if (m_autoSelected == TEST)
  {
    deployDirectory = "Test";
  }

  m_autoAction = m_autoSequence->front();

  std::cout << "auto chooser\n";

  Trajectory::follow(deployDirectory);
  std::cout << "directory deployed \n";
  Drivetrain::stop();
  std::cout << "drivetrian stop \n";

  // If driving after "stop" is called is a problem, I will add a "stop" method
  //  which runs a few times to ensure all modules are stopped

  // Will only finish after trajectory is done, so we can add additional trajectories and timers to intake & shoot
}

void Robot::AutonomousPeriodic()
{
  // This is what gets called after Init()
  Drivetrain::stop();
}
// File
fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

//  Drivetrain::stop();

void Robot::TeleopInit()
{
  std::cout << "TeleopInit";

   m_grippad.retract();
}

void Robot::TeleopPeriodic()
{
  // DASHBOARD::update_botpose(m_camera.get_field_pos_by_tag());
  // Drivetrain::print_angle();
  //  m_camera.pose_loop();
  buttonManager();
  swerveDrive(field_centric);

  if (BUTTON::DRIVETRAIN::ZERO())
  {
    Drivetrain::zero_yaw();
  }
  Odometry::update(); 

  if constexpr (debugging)
  {
    Trajectory::printRobotRelativeSpeeds();
    Trajectory::printFieldRelativeSpeeds();
  }

  if (BUTTON::GRIPPADS::GRIPPADS_RETRACT())
  {
    std::cout << "retracting...\n";
    m_grippad.retract();
  }
  else if (BUTTON::GRIPPADS::GRIPPADS_DEPLOY())
  {
    m_grippad.deploy();
  }

  if (BUTTON::ARM::ARM_STORED())
  {
    state = CONSTANTS::STATES::STORED;
  }
  else if (BUTTON::ARM::ARM_HP())
  {
    state = CONSTANTS::STATES::HUMANPLAYER;
  }
  else if (BUTTON::ARM::ARM_PICKUP())
  {
    state = CONSTANTS::STATES::PICKUP;
  }
  else if (BUTTON::ARM::ARM_LOW())
  {
    state = CONSTANTS::STATES::LOW;
  }
  else if (BUTTON::ARM::ARM_MID())
  {
    state = CONSTANTS::STATES::MED;
  }
  else if (BUTTON::ARM::ARM_HIGH())
  {
    state = CONSTANTS::STATES::HIGH;
  }
  else if ((state == CONSTANTS::STATES::SCORE && BUTTON::DRIVETRAIN::ABORT()) ||
    ((state == CONSTANTS::STATES::HUMANPLAYER && BUTTON::DRIVETRAIN::ABORT())))
  {
    state = CONSTANTS::STATES::ABORT;
    m_force_pos = m_arm.Read_Position() + 112;
  }
  else if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_HP())
  {
    state = CONSTANTS::STATES::O_HP;
  }
  // else if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_PICKUP())
  // {
  //   state = CONSTANTS::STATES::PICKUP;
  // }
  else if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_LOW())
  {
    state = CONSTANTS::STATES::O_LOW;
  }
  else if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_MID())
  {
    state = CONSTANTS::STATES::O_MED;
  }
  else if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_HIGH())
  {
    state = CONSTANTS::STATES::O_HIGH;
  }

  switch (state)
  {
  case CONSTANTS::STATES::STORED:
    m_grabber.close();
    m_arm.arm_moved(state);
    break;

  case CONSTANTS::STATES::HUMANPLAYER:
    if (m_arm.arm_moved(state))
    {
      // std::cout << "Within \n";
      m_grabber.open();
      m_robot_timer.Start();
      // std::cout << "Break Beam: " << m_grabber.break_beam() <<std::endl;
      if ((!m_grabber.break_beam() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(1.0))
      {
        m_grabber.close();
        if (m_robot_timer.Get() > units::time::second_t(1.5))
        {
          m_robot_timer.Stop();
          m_robot_timer.Reset();
          m_arm.arm_moved(CONSTANTS::STATES::STORED);
          state = CONSTANTS::STATES::STORED;
        }
      }
    }
    break;

  case CONSTANTS::STATES::PICKUP:
    // std::cout << "Robot state: Pickup \n";
    if (m_arm.arm_moved(state))
    {
      // std::cout << "Within \n";
      m_grabber.open();
      m_robot_timer.Start();
      // std::cout << "Break Beam: " << m_grabber.break_beam() <<std::endl;
      if ((!m_grabber.break_beam() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(1.0))
      {
        m_robot_timer.Stop();
        m_robot_timer.Reset();
        m_grabber.close();
        m_arm.arm_moved(CONSTANTS::STATES::STORED);
        state = CONSTANTS::STATES::STORED;
      }
    }
    break;

  case CONSTANTS::STATES::LOW:
    if (m_camera.pose_loop())
    {
      if (m_arm.arm_moved(state))
      {
        Robot::traj_init(Trajectory::HEIGHT::GROUND);
        state = CONSTANTS::STATES::SCORE;
      }
    }
    break;

  case CONSTANTS::STATES::MED:
    if (m_camera.pose_loop())
    {
      if (m_arm.arm_moved(state))
      {
        Robot::traj_init(Trajectory::HEIGHT::MED);
        state = CONSTANTS::STATES::SCORE;
      }
    }
    break;

  case CONSTANTS::STATES::HIGH:
    if (m_camera.pose_loop())
    {
      if (m_arm.arm_moved(state))
      {
        Robot::traj_init(Trajectory::HEIGHT::HIGH);
        state = CONSTANTS::STATES::SCORE;
      }
    }
    break;

  case CONSTANTS::STATES::FALLBACK:
    if (Trajectory::follow_live_traj(m_back_trajectory))
    {
      m_robot_timer.Stop();
      m_robot_timer.Reset();
      m_grabber.close();
      m_arm.arm_moved(CONSTANTS::STATES::STORED);
      state = CONSTANTS::STATES::STORED;
    }
    break;

  case CONSTANTS::STATES::ABORT:
    m_arm.arm_moved(CONSTANTS::STATES::ABORT);
    m_arm.force_move(m_force_pos);
    m_grabber.close();
    m_robot_timer.Stop();
    m_robot_timer.Reset();
    break;

  case CONSTANTS::STATES::SCORE:

    if (Trajectory::follow_live_traj(m_trajectory))
    {
      m_grabber.open();
      m_robot_timer.Start();

      if (m_robot_timer.Get() > units::time::second_t(0.5))
      {
        m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back());
        Trajectory::init_live_traj(m_back_trajectory);
        state = CONSTANTS::STATES::FALLBACK;
      }
    }
    break;

    case CONSTANTS::STATES::O_HP:
      if (m_arm.arm_moved(CONSTANTS::STATES::HUMANPLAYER))
      {
      m_grabber.open();
      m_robot_timer.Start();
      if ((!m_grabber.break_beam() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(0.5))
      {
        m_grabber.close();
      }
    }
    break;

    case CONSTANTS::STATES::O_LOW:
      if (m_arm.arm_moved(CONSTANTS::STATES::LOW))
      {
        if (BUTTON::GRABBER::OVERIDE_TOGGLE())
        {
          m_grabber.open();
        }
      }
      break;

      case CONSTANTS::STATES::O_MED:
        if (m_arm.arm_moved(CONSTANTS::STATES::MED))
        {
          if (BUTTON::GRABBER::OVERIDE_TOGGLE())
          {
            m_grabber.open();
          }
        }
        break;

        case CONSTANTS::STATES::O_HIGH:
        if (m_arm.arm_moved(CONSTANTS::STATES::HIGH))
        {
          if (BUTTON::GRABBER::OVERIDE_TOGGLE())
          {
            m_grabber.open();
          }
        }
        break;
  }

  m_candle.candle_logic(BUTTON::CANDLE::CANDLE_LEFT(), BUTTON::CANDLE::CANDLE_RIGHT(), BUTTON::CANDLE::CANDLE_YELLOW(), BUTTON::CANDLE::CANDLE_PURPLE());
}

void Robot::traj_init(Trajectory::HEIGHT h)
{
  std::cout << "here 0 \n";
  switch (h)
  {
  case Trajectory::HEIGHT::HIGH:
    std::cout << "high \n";
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::HIGH));
    break;
  case Trajectory::HEIGHT::MED:
    std::cout << "mid \n";
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::MED));
    break;
  case Trajectory::HEIGHT::GROUND:
    std::cout << "gnd \n";
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::GROUND));
  }
  std::cout << "here 1 \n";
  Trajectory::init_live_traj(m_trajectory);
  std::cout << "here 2 \n";
}

void Robot::make_test_path()
{
  // frc::Pose2d current_pose = Odometry::getPose();
  frc::Pose2d current_pose(frc::Translation2d(5.0_m, 0.0_m), frc::Rotation2d(Drivetrain::getCCWHeading()));

  auto x = 0.5_m;
  auto y = -2.0_m;

  auto heading = (frc::Translation2d(x, y) - current_pose.Translation()).Angle().Degrees();

  m_trajectory = Trajectory::generate_live_traj(current_pose.X(),
                                                current_pose.Y(),
                                                frc::Rotation2d(heading /*current_pose.Rotation().Degrees()*/),
                                                frc::Rotation2d(current_pose.Rotation().Degrees()),
                                                current_pose.X() + x,
                                                current_pose.Y() + y,
                                                frc::Rotation2d(heading),
                                                frc::Rotation2d(0.0_deg /*current_pose.Rotation().Degrees()*/)
                                                // frc::Rotation2d(Drivetrain::getCCWHeading()),
                                                // frc::Rotation2d(Drivetrain::getCCWHeading())
  );
}
void Robot::TestInit()
{
  std::cout << "Test init \n";
  Odometry::update();
  make_test_path();
  Trajectory::init_live_traj(m_trajectory);
  m_is_auto = false;
}

void Robot::TestPeriodic()
{
  // auto pose = Odometry::getPose();
  // std::cout << "Navx " << Drivetrain::getAngle().value() << std::endl;
  Trajectory::follow_live_traj(m_trajectory);
}

void Robot::DisabledPeriodic()
{
  m_candle.RainbowAnim();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
