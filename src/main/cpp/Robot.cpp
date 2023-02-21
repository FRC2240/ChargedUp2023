#include "Robot.hpp"
#include "Trajectory.hpp"


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
//static bool drivers_flashing_red = false;
//static bool turners_flashing_red = false;

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
}


void buttonManager()
{
    if (BUTTON::DRIVETRAIN::FIELD_CENTRIC())
      {
      std::cout << "MODE: RT" << "\n";
      field_centric = !field_centric;
      }
}

void tankDrive(){}

void tunePID(){}


void tuneFF(){}


void swerveDrive(bool const &field_relative)
{
  auto const left_right = -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), m_deadband) *
    Drivetrain::TELEOP_MAX_SPEED;

  auto const front_back = -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), m_deadband) *
    Drivetrain::TELEOP_MAX_SPEED;
  //std::cout << "left_right = " << BUTTON::PS5.GetX() << "front_back = " << BUTTON::PS5.GetY() << std::endl;
  
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
    auto const rot = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), m_deadband) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, -left_right, rot, field_relative);
  }
}

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
void Robot::RobotPeriodic()
{
  Trajectory::reverse_trajectory = frc::SmartDashboard::GetBoolean("Traj Reversed", Trajectory::reverse_trajectory);
  //std::cout << "Robot Periodic \n";

  // if (m_arm.position > m_arm.ARM_FLARE_LOW && m_arm.position < m_arm.ARM_FLARE_HIGH){
  //   m_wrist.Follow_Flare(m_arm.position);
  // }
  // else {
  //std::cout << m_arm.position << std::endl;
  m_wrist.Follow(m_arm.position);
  // }
  m_arm.Read_Position();
}

void Robot::AutonomousInit()
{

    if(m_autoSelected == TEST){
      m_autoSequence = &m_testSequence;
    }

    m_autoAction = m_autoSequence->front();

  // Start aiming

  //m_autoSelected = m_chooser.GetSelected();
  std::cout << "auto selected \n";

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  if (m_autoSelected == CIRCLE)
    {
      deployDirectory =  "Circle";
    }

  if (m_autoSelected == LINE)
    {
      deployDirectory = "30 degree turn";
    }

 if (m_autoSelected == LINE)
   {
      deployDirectory = "Straight Line";
   }

   if(m_autoSelected == TEST){
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
}

void Robot::TeleopPeriodic()
{
  //DASHBOARD::update_botpose(m_camera.get_field_pos_by_tag());
  //Drivetrain::print_angle();
 // m_camera.pose_loop();
  buttonManager();
  swerveDrive(field_centric);
  //Odometry::update();

  if constexpr (debugging)
    {
      Trajectory::printRobotRelativeSpeeds();
      Trajectory::printFieldRelativeSpeeds();
    }

  if (breakbeam == false && m_grabber.m_BreakBeamSensor.Get() == true ){
    breakbeam = true;
  }

  if (m_grabber.grabberToggle == false && BUTTON::GRABBER::GRABBER_TOGGLE())
    {
      m_grabber.In();
      m_grabber.grabberToggle = true;
    }
  else if ((m_grabber.grabberToggle == true && BUTTON::GRABBER::GRABBER_TOGGLE()) || (m_grabber.m_BreakBeamSensor.Get() == false && breakbeam == true))
    {
      m_grabber.Out();
      m_grabber.grabberToggle = false; 
      breakbeam = false;
    }

  m_candle.candle_logic(BUTTON::CANDLE::CANDLE_LEFT(), BUTTON::CANDLE::CANDLE_RIGHT(), BUTTON::CANDLE::CANDLE_YELLOW(), BUTTON::CANDLE::CANDLE_PURPLE());


  m_arm.arm_moved(BUTTON::ARM::ARM_STORED(), BUTTON::ARM::ARM_LOW(), 
                  BUTTON::ARM::ARM_MID(), BUTTON::ARM::ARM_HP(), 
                  BUTTON::ARM::ARM_HIGH(), BUTTON::ARM::ARM_PICKUP());
  arm_bool = m_arm.open_grabber;
  m_grabber.GrabberLogic(arm_bool);

}

void Robot::make_test_path()
{
  frc::Pose2d current_pose = Odometry::getPose();

  auto heading = (frc::Translation2d(1_m, 1_m) - current_pose.Translation()).Angle().Degrees();

  m_trajectory = Trajectory::generate_live_traj(current_pose.X(),
                                 current_pose.Y(),
                                frc::Rotation2d(heading/*current_pose.Rotation().Degrees()*/),
                                 frc::Rotation2d(current_pose.Rotation().Degrees()),
                                 current_pose.X() + 0.25_m,
                                 current_pose.Y() + 0.5_m,
                                 frc::Rotation2d(heading),
                                 frc::Rotation2d(0.0_deg/*current_pose.Rotation().Degrees()*/)
                                //frc::Rotation2d(Drivetrain::getCCWHeading()),
                                 //frc::Rotation2d(Drivetrain::getCCWHeading())
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
  //auto pose = Odometry::getPose();
  //std::cout << "Navx " << Drivetrain::getAngle().value() << std::endl;
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
