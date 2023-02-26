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
  m_chooser.AddOption(AUTO_STATION, AUTO_STATION);
  m_chooser.AddOption(AUTO_LINE, AUTO_LINE);
  m_chooser.AddOption(AUTO_NOTHING, AUTO_NOTHING);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  std::cout << "robot object created \n";
  std::cout << "go get em tiger" << std::endl;
}

void Robot::RobotInit()
{
  Odometry::putField2d();
  std::cout << "RobotInit done \n";

  m_grippad.retract();

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();
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


    auto const rot = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, -left_right, rot, field_relative);

    frc::SmartDashboard::PutNumber("Gyro: ", Drivetrain::getAngle().value());
    frc::SmartDashboard::PutNumber("front/back: ", front_back.value());
    frc::SmartDashboard::PutNumber("left/right: ", left_right.value());


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
  //m_wrist.Follow(m_arm.position);
  // }
  m_arm.Read_Position();
}

void Robot::AutonomousInit()
{
  m_grippad.retract();

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == AUTO_STATION) {
    state = CONSTANTS::STATES::HIGH;
    m_fallback_pos = 9.9_ft;
  } else if (m_autoSelected == AUTO_LINE) {
    state = CONSTANTS::STATES::HIGH;
    m_fallback_pos = 12.0_ft;
  } else {
    state = CONSTANTS::STATES::STORED;
  }
}

void Robot::AutonomousPeriodic()
{
  m_wrist.Follow(m_arm.position);

  switch (state)
  {
  case CONSTANTS::STATES::STORED:
    m_grabber.close();
    m_arm.arm_moved(state);
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

  case CONSTANTS::STATES::SCORE:
    if (Trajectory::follow_live_traj(m_trajectory))
    {
      m_grabber.open();
      m_robot_timer.Start();

      if (m_robot_timer.Get() > units::time::second_t(0.5))
      {
        m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back(m_fallback_pos));
        Trajectory::init_live_traj(m_back_trajectory);
        state = CONSTANTS::STATES::FALLBACK;
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
  default:
    break;
  }
}
// File
//fs::path deployDirectory = frc::filesystem::GetDeployDirectory();

//  Drivetrain::stop();

void Robot::TeleopInit()
{
  std::cout << "TeleopInit";
  state = CONSTANTS::STATES::STORED;


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
    state = CONSTANTS::STATES::O_HP;
  }
  else if (BUTTON::ARM::ARM_PICKUP())
  {
    state = CONSTANTS::STATES::PICKUP;
  }
  else if (BUTTON::ARM::ARM_LOW())
  {
    state = CONSTANTS::STATES::O_LOW;
    // std::cout << "start: " << Odometry::getPose().X().value() << 
    // " , " <<
    //  Odometry::getPose().Y().value() <<
    //  std::endl;
  }
  else if (BUTTON::ARM::ARM_MID())
  {
    state = CONSTANTS::STATES::O_MED;
    //  std::cout << "start: " << Odometry::getPose().X().value() << 
    // " , " <<
    //  Odometry::getPose().Y().value() <<
    //  std::endl;
  }
  else if (BUTTON::ARM::ARM_HIGH())
  {
    state = CONSTANTS::STATES::O_HIGH;
    // std::cout << "start: " << Odometry::getPose().X().value() << 
    // " , " <<
    //  Odometry::getPose().Y().value() <<
    //  std::endl;
  }
  else if ((state == CONSTANTS::STATES::SCORE && BUTTON::DRIVETRAIN::ABORT()) ||
    ((state == CONSTANTS::STATES::HUMANPLAYER && BUTTON::DRIVETRAIN::ABORT())))
  {
    state = CONSTANTS::STATES::ABORT;
    m_force_pos = m_arm.Read_Position();
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

  if (state == CONSTANTS::STATES::O_HIGH || state == CONSTANTS::STATES::O_LOW || state == CONSTANTS::STATES::O_MED || state == CONSTANTS::STATES::STORED || state == CONSTANTS::STATES::SCORE){
    m_wrist.Follow(m_arm.position);
  }
  else if (state == CONSTANTS::STATES::O_HP || state == CONSTANTS::STATES::PICKUP){
    m_wrist.pickupFollow(m_arm.position);
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
      if (BUTTON::GRABBER::TOGGLE() && m_robot_timer.Get() > units::time::second_t(1.0))
      {
        //std::cout << "beam: " << m_grabber.break_beam() << "toggle: " << BUTTON::GRABBER::TOGGLE() << "\n";
        m_grabber.close();
        m_robot_timer2.Start();
        if (m_robot_timer2.Get() > units::time::second_t(0.5))
        {
          m_robot_timer.Stop();
          m_robot_timer.Reset();
          m_robot_timer2.Stop();
          m_robot_timer2.Reset();
          m_arm.arm_moved(CONSTANTS::STATES::STORED);
          state = CONSTANTS::STATES::STORED;
        }
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
        std::cout << "end: " << Odometry::getPose().X().value() << 
        " , " <<
        Odometry::getPose().Y().value() <<
        std::endl;
        m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back(1.0_m));
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
      if (BUTTON::GRABBER::TOGGLE() && m_robot_timer.Get() > units::time::second_t(0.5))
      {
        m_grabber.close();
        m_robot_timer2.Start();
        if (m_robot_timer2.Get() > units::time::second_t(1.0))
        {
          m_robot_timer2.Stop();
          m_robot_timer2.Reset();
          m_robot_timer.Stop();
          m_robot_timer.Reset();
          m_arm.arm_moved(CONSTANTS::STATES::STORED);
          state = CONSTANTS::STATES::STORED;
        }
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

  m_candle.candle_logic(BUTTON::CANDLE::CANDLE_LEFT(), BUTTON::CANDLE::CANDLE_RIGHT(), BUTTON::CANDLE::CANDLE_YELLOW(), BUTTON::CANDLE::CANDLE_PURPLE(), m_grabber.grabberStatus());

  //std::cout << m_arm.position << std::endl;
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
}

void Robot::TestPeriodic()
{
  m_arm.test();
  m_wrist.test();
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
