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
  Odometry::putField2d();

  // Auto paths
  m_chooser.AddOption(AUTO_STATION, AUTO_STATION);
  m_chooser.AddOption(AUTO_LINE, AUTO_LINE);
  m_chooser.AddOption(AUTO_NOTHING, AUTO_NOTHING);
  m_chooser.AddOption(AUTO_BALANCE, AUTO_BALANCE);
  m_chooser.AddOption(SCORE_IDLE, SCORE_IDLE);
  m_chooser.AddOption(HP_CONE,HP_CONE);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotInit()
{
  Odometry::putField2d();

  m_grippad.retract();

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();
}

void buttonManager()
{
  if (BUTTON::DRIVETRAIN::FIELD_CENTRIC())
  {
    field_centric = !field_centric;
  }
}

void tankDrive() {}

void tunePID() {}

void tuneFF() {}

void swerveDrive(bool const &field_relative)
{
  const auto left_right = (BUTTON::DRIVETRAIN::TURBO()) ? -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND) *
                          Drivetrain::TELEOP_MAX_SPEED : -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND) * 
                          CONSTANTS::NON_TURBO*
                          Drivetrain::TELEOP_MAX_SPEED;
                          
  const auto front_back = (BUTTON::DRIVETRAIN::TURBO()) ? -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND) *
                          Drivetrain::TELEOP_MAX_SPEED : -frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND) * 
                          CONSTANTS::NON_TURBO*
                          Drivetrain::TELEOP_MAX_SPEED;
  // std::cout << "left_right = " << BUTTON::PS5.GetX() << "front_back = " << BUTTON::PS5.GetY() << std::endl;
  /*if (!BUTTON::DRIVETRAIN::TURBO())
  {
    left_right = left_right * CONSTANTS::NON_TURBO;
    front_back = front_back * CONSTANTS::NON_TURBO;
  }*/

    auto const rot = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND) * Drivetrain::TELEOP_MAX_ANGULAR_SPEED;

    Drivetrain::drive(front_back, -left_right, rot, field_relative);

    // frc::SmartDashboard::PutNumber("Gyro: ", Drivetrain::getAngle().value());
    // frc::SmartDashboard::PutNumber("front/back: ", front_back.value());
    // frc::SmartDashboard::PutNumber("left/right: ", left_right.value());


}

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/
void Robot::RobotPeriodic()
{
  //Trajectory::reverse_trajectory = frc::SmartDashboard::GetBoolean("Traj Reversed", Trajectory::reverse_trajectory);
  m_arm.read_position();
}

void Robot::AutonomousInit()
{
  //std::cout << Drivetrain::getAngle().value() << std::endl;
  //std::cout << Drivetrain::get_offset() << std::endl;

  Odometry::update();
  Drivetrain::flip();
  m_grippad.retract();
  //  std::cout << Drivetrain::getAngle().value() << std::endl;
  //std::cout << Drivetrain::get_offset() << std::endl;

  // Get choosen autonomous mode
  m_autoSelected = m_chooser.GetSelected();

  if (m_autoSelected == AUTO_STATION) 
  {
    m_autoSequence = &m_score_and_balance_sequence;
    m_fallback_pos = 12.5_ft;
  } 
  else if (m_autoSelected == AUTO_LINE) 
  {
    m_autoSequence = &m_score_and_leave_sequence;
    m_fallback_pos = 12.5_ft;
  } 
  else if (m_autoSelected == AUTO_BALANCE)
  {
    m_autoSequence = &m_balance_sequence;
  }
  else if (m_autoSelected == SCORE_IDLE)
  {
    m_autoSequence = &m_score_and_idle_sequence;
  }
  else if (m_autoSelected == HP_CONE) 
  {
    m_autoSequence = &m_HP_cone_sequence;
  } 
  else 
  {
    state = CONSTANTS::STATES::STORED;
  }

  m_autoAction = m_autoSequence->front();
  m_autoState = kNothing;
}

void Robot::AutonomousPeriodic()
{
    // std::cout << Drivetrain::getAngle().value() << std::endl;
  // std::cout << Drivetrain::get_offset() << std::endl;
  m_wrist.follow(m_arm.position);
  switch(m_autoAction) {

  case kBalance:
    m_autoAction = kIdle;
    m_autoState = kBalancing;
    break;

  case kBackwardsBalance:
    m_autoAction = kIdle;
    m_autoState = kBackwardsBalancing;
    break;

  case kScore:
      m_candle.bounce_anim();
    if (m_arm.arm_moved(CONSTANTS::STATES::HIGH))
      {
        if (m_camera.pose_loop())
          {
            Robot::traj_init(Trajectory::HEIGHT::HIGH);
            m_autoAction = kScore_periodic;

          }
      }
    break;

  case kScoreLow:
    if (m_arm.arm_moved(CONSTANTS::STATES::LOW))
    {
      m_grabber.open();
      m_arm.arm_moved(CONSTANTS::STATES::STORED);
      if (m_arm.position < 56.0) 
      {
            m_autoSequence->pop_front();
            m_autoAction = m_autoSequence->front();
            m_autoState = kNothing;
      }
    }
    break;

  case kScore_periodic:
    if (Trajectory::follow_live_traj(m_trajectory))
      {
        m_grabber.open();
        m_robot_timer.Start();

        if (m_robot_timer.Get() > 0.5_s)
          {
            m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back(0.6_m));
            Trajectory::init_live_traj(m_back_trajectory);
            m_autoAction = kAutoFallback;
          }
      }
    break;

    case kAutoFallback:
      if (Trajectory::follow_live_traj(m_back_trajectory))
        {
          m_robot_timer.Stop();
          m_robot_timer.Reset();
          m_grabber.close();
          m_arm.arm_moved(CONSTANTS::STATES::STORED);
          if (m_arm.position < 56.0) {
            m_autoSequence->pop_front();
            m_autoAction = m_autoSequence->front();
            m_autoState = kNothing;
          }
        }
      break;

    case kFallbackPath:
      m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back(m_fallback_pos));
      Trajectory::init_live_traj(m_back_trajectory);
      m_autoAction = kFallbackPathPeriodic;
      break;

    case kFallbackPathPeriodic:
      if (Trajectory::follow_live_traj(m_back_trajectory))
      {
        m_autoSequence->pop_front();
        m_autoAction = m_autoSequence->front();
        m_autoState = kNothing;
      }
      break;

    case kIdle:
      break;

    case kHPConePath1:
      std::cout << "path 1 \n";
      m_path_trajectory1 = Trajectory::extract("3_cone_HP_side_1");
      Trajectory::init_live_traj(m_path_trajectory1);
      m_autoAction = kHPConePath1_periodic;
      break;

    case kHPConePath1_periodic:
      std::cout << "driving path 1 \n";
      m_autoState = kIntaking;
      if (Trajectory::follow_live_traj(m_path_trajectory1))
        {
          // m_autoSequence->pop_front();
          // m_autoAction = m_autoSequence->front();
          //m_autoState = kNothing;
        }
      break;

      case kHPConePath2:
        std::cout << "path 2 \n";
        m_path_trajectory2 = Trajectory::extract("3_cone_HP_side_2");
        Trajectory::init_live_traj(m_path_trajectory2);
        m_autoAction = kHPConePath2_periodic;
        break;

    case kHPConePath2_periodic:
      std::cout << "driving path 2 \n";
      m_autoState = kNothing;
      if (Trajectory::follow_live_traj(m_path_trajectory2))
      {
        m_autoSequence->pop_front();
        m_autoAction = m_autoSequence->front();
        m_autoState = kNothing;
      }
      break;

    case kDelay:
      std::cout << "Delay\n";
      m_robot_timer2.Start();
      if (m_robot_timer2.Get() > units::time::second_t(1.0)){
        m_autoSequence->pop_front();
        m_autoAction = m_autoSequence->front();
        m_robot_timer2.Stop();
        m_robot_timer2.Reset();
      }
      break;
  }

  if (m_autoState == kBalancing) 
  {
    speed = m_auto_balance.auto_balance_routine();
    Drivetrain::faceDirection(-speed * Drivetrain::ROBOT_MAX_SPEED, 0_mps, 180_deg, false, 0.0);
  }
  else if (m_autoState == kBackwardsBalancing)
  {
    speed = m_auto_balance.auto_balance_routine_backwards();
    Drivetrain::faceDirection(speed * Drivetrain::ROBOT_MAX_SPEED, 0_mps, 0_deg, false, 0.0);
  }

  if (m_autoState == kIntaking) {
    if (m_arm.arm_moved(CONSTANTS::STATES::PICKUP))
      {
        m_wrist.pickup();
        m_grabber.open();
        m_robot_timer.Start();
        if ((!m_grabber.limit_switch() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(1.0))
        {
          m_grabber.set_sensor(false);
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
            m_autoSequence->pop_front();
            m_autoAction = m_autoSequence->front();
            m_autoState = kNothing;
          }
        }
      }
      else 
      {
        m_wrist.follow(m_arm.position);
      }
  }

}

void Robot::TeleopInit()
{
  Odometry::update();
  state = CONSTANTS::STATES::STORED;
  //std::cout << "navx " << Drivetrain::getCCWHeading().Degrees().value() << std::endl;
  //std::cout << "odometry: " << Odometry::getPose().Rotation().Degrees().value() << std::endl;

   m_grippad.retract();
}

void Robot::TeleopPeriodic()
{

  // DASHBOARD::update_botpose(m_camera.get_field_pos_by_tag());
  // Drivetrain::print_angle();
  //  m_camera.pose_loop();
  buttonManager();
  swerveDrive(field_centric);

  if (BUTTON::m_aux_stick.GetStartButton())
  {
    m_camera.pose_loop();
  }

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
    //std::cout << "retracting...\n";
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
    m_grabber.set_sensor(true);
  }
  else if (BUTTON::ARM::ARM_PICKUP())
  {
    state = CONSTANTS::STATES::PICKUP;
    m_grabber.set_sensor(true);
  }
  else if (BUTTON::ARM::ARM_LOW())
  {
    state = CONSTANTS::STATES::O_LOW;
  }
  else if (BUTTON::ARM::ARM_MID())
  {
    state = CONSTANTS::STATES::O_MID;
  }
  else if (BUTTON::ARM::ARM_HIGH())
  {
    state = CONSTANTS::STATES::O_HIGH;
  }
  else if ((state == CONSTANTS::STATES::SCORE && BUTTON::DRIVETRAIN::ABORT()) ||
    ((state == CONSTANTS::STATES::HIGH && BUTTON::DRIVETRAIN::ABORT())) ||
    ((state == CONSTANTS::STATES::MID && BUTTON::DRIVETRAIN::ABORT())) ||
    ((state == CONSTANTS::STATES::LOW && BUTTON::DRIVETRAIN::ABORT())) ||
    ((state == CONSTANTS::STATES::HP_AUTO && BUTTON::DRIVETRAIN::ABORT())))

  {
    state = CONSTANTS::STATES::ABORT;
    m_force_pos = m_arm.desired_position;
  }
  
  else if (BUTTON::ARM::OVERIDES::PICKUP_OVERIDE())
  {
    state = CONSTANTS::STATES::O_PICKUP;
    m_grabber.set_sensor(true);
  }
  else if (BUTTON::ARM::OVERIDES::HP_OVERIDE())
  {
    state = CONSTANTS::STATES::O_HP_PICKUP;
    m_grabber.set_sensor(true);
  }

  if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_OPEN())
  {
    state = CONSTANTS::STATES::O_OPEN;
  }
  if (BUTTON::ARM::OVERIDES::ARM_OVERIDE_UP())
  {
    state = CONSTANTS::STATES::O_UP;
    m_grabber.set_sensor(true);
  }

  if (state == CONSTANTS::STATES::O_HIGH || 
      state == CONSTANTS::STATES::LOW ||
      state == CONSTANTS::STATES::MID ||
      state == CONSTANTS::STATES::HIGH ||
      state == CONSTANTS::STATES::O_LOW ||
      state == CONSTANTS::STATES::O_MID || 
      state == CONSTANTS::STATES::STORED || 
      state == CONSTANTS::STATES::SCORE || 
      state == CONSTANTS::STATES::O_UP || 
      state == CONSTANTS::STATES::O_OPEN)
  {
    m_wrist.follow(m_arm.position);
  }
  // else if (state == CONSTANTS::STATES::O_HP || state == CONSTANTS::STATES::PICKUP){
  //   m_wrist.pickupFollow(m_arm.position);
  // }

  switch (state)
  {
  case CONSTANTS::STATES::STORED:
      m_robot_timer.Stop();
      m_robot_timer.Reset();
      m_grabber.close();
      m_arm.arm_moved(state);
      break;

    case CONSTANTS::STATES::HP:
     if (m_arm.arm_moved(state))
      {
        m_grabber.set_sensor(false);
       m_grabber.open();
        m_robot_timer.Start();
        if ((!m_grabber.limit_switch() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(1.0))
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

    case CONSTANTS::STATES::HP_AUTO:
      m_candle.bounce_anim();
      if (m_grabber.limit_switch())
      {
        m_grabber.open();
        Drivetrain::faceDirection(CONSTANTS::TRAJECTORY::HP_VEL, 0_mps, Odometry::getPose().Rotation().Degrees(), false, 0.0);
      }
      else
      {
        m_grabber.close();
        m_grabber.set_sensor(false);
      }
    break;

    case CONSTANTS::STATES::PICKUP:
      if (m_arm.arm_moved(state))
      {
        m_wrist.pickup();
        m_grabber.open();
        m_robot_timer.Start();
        if ((!m_grabber.limit_switch() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(1.0))
        {
          m_grabber.set_sensor(false);
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
      else 
      {
        m_wrist.follow(m_arm.position);
      }
      break;

  case CONSTANTS::STATES::LOW:
    m_candle.bounce_anim();
    if (Drivetrain::snap_to_zero())
      {
        if (m_camera.pose_loop())
          {
            Robot::traj_init(Trajectory::HEIGHT::GROUND);
            state = CONSTANTS::STATES::SCORE;
          }
      }
    break;

  case CONSTANTS::STATES::MID:
    m_candle.bounce_anim();
    if (Drivetrain::snap_to_zero())
      {
        if (m_camera.pose_loop())
          {
            Robot::traj_init(Trajectory::HEIGHT::MED);
            state = CONSTANTS::STATES::SCORE;
          }
      }
    break;

  case CONSTANTS::STATES::HIGH:
    m_candle.bounce_anim();
    if (Drivetrain::snap_to_zero())
      {
        if (m_camera.pose_loop())
            {
              Robot::traj_init(Trajectory::HEIGHT::HIGH);
              state = CONSTANTS::STATES::SCORE;
            }
        }
        break;

    case CONSTANTS::STATES::FALLBACK:
      m_candle.bounce_anim();
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
        m_candle.bounce_anim();
        if (Trajectory::follow_live_traj(m_trajectory))
        {
          m_grabber.open();
          m_robot_timer.Start();
          if (m_robot_timer.Get() > 0.5_s)
            {
              //std::cout << "end: " << Odometry::getPose().X().value() <<
              //" , " <<
              //Odometry::getPose().Y().value() <<
              //std::endl;
              // m_back_trajectory = Trajectory::generate_live_traj(Trajectory::fall_back(1.0_m));
              // Trajectory::init_live_traj(m_back_trajectory);
              m_force_pos = m_arm.read_position();
              state = CONSTANTS::STATES::ABORT;
            }
        }
        break;

      case CONSTANTS::STATES::O_HP:
        if (m_arm.arm_moved(CONSTANTS::STATES::HP))
          {
             if (BUTTON::ARM::TRIGGER_AUTO())
              {
                state = CONSTANTS::STATES::HP_AUTO;
              }
 
            m_wrist.human_player();
            m_grabber.open();
            m_robot_timer.Start();
            if ((!m_grabber.limit_switch() || BUTTON::GRABBER::TOGGLE()) && m_robot_timer.Get() > units::time::second_t(0.5))
              {
                m_grabber.close();
                m_robot_timer.Stop();
                m_robot_timer.Reset();
                last_state = CONSTANTS::STATES::HP;
                state = CONSTANTS::STATES::IDLE;

              }
          }
        else 
        {
        m_wrist.follow(m_arm.position);
        }
        break;

      case CONSTANTS::STATES::O_LOW:
        if (m_arm.arm_moved(CONSTANTS::STATES::LOW))
          {
            if (BUTTON::ARM::TRIGGER_AUTO())
              {
                state = CONSTANTS::STATES::LOW;
              }
            if (BUTTON::GRABBER::OVERIDE_TOGGLE())
              {
                m_grabber.open();
              }
          }
        break;

      case CONSTANTS::STATES::O_MID:
        if (m_arm.arm_moved(CONSTANTS::STATES::MID))
          {
            if (BUTTON::ARM::TRIGGER_AUTO())
              {
                state = CONSTANTS::STATES::MID;
              }
            if (BUTTON::GRABBER::OVERIDE_TOGGLE())
              {
                m_grabber.open();
              }
          }
        break;

      case CONSTANTS::STATES::O_HIGH:
        if (m_arm.arm_moved(CONSTANTS::STATES::HIGH))
          {
            if (BUTTON::ARM::TRIGGER_AUTO())
              {
                state = CONSTANTS::STATES::HIGH;
              }
            if (BUTTON::GRABBER::OVERIDE_TOGGLE())
              {
                m_grabber.open();
              }
          }
        break;

      case CONSTANTS::STATES::IDLE:
        m_arm.arm_moved(last_state);
        break;


      case CONSTANTS::STATES::O_OPEN:
        m_grabber.open();
        break;

      case CONSTANTS::STATES::O_UP:
        m_arm.arm_moved(state);
        break;

      case CONSTANTS::STATES::O_PICKUP:
        if (m_arm.arm_moved(CONSTANTS::STATES::PICKUP))
        {
          m_wrist.pickup();
          m_grabber.open();
          m_robot_timer.Start();
          if (BUTTON::GRABBER::TOGGLE())
          {
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
        else 
        {
          m_wrist.follow(m_arm.position);
        }
        break;

      case CONSTANTS::STATES::O_HP_PICKUP:
        if (m_arm.arm_moved(CONSTANTS::STATES::HP))
        {
          m_wrist.human_player();
          m_grabber.open();
          m_robot_timer.Start();
          if (BUTTON::GRABBER::TOGGLE())
          {
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
        else 
        {
          m_wrist.follow(m_arm.position);
        }
        break;

      }
      
      if (
        state != CONSTANTS::STATES::HIGH &&
        state != CONSTANTS::STATES::MID &&
        state != CONSTANTS::STATES::LOW &&
        state != CONSTANTS::STATES::SCORE &&
        state != CONSTANTS::STATES::FALLBACK &&
        state != CONSTANTS::STATES::HP_AUTO
      )
      {
          m_candle.candle_logic(BUTTON::CANDLE::CANDLE_LEFT(),
                        BUTTON::CANDLE::CANDLE_RIGHT(), 
                        BUTTON::CANDLE::CANDLE_YELLOW(), 
                        BUTTON::CANDLE::CANDLE_PURPLE(), 
                        m_grabber.grabber_status());
      }
  }



void Robot::traj_init(Trajectory::HEIGHT h)
{
  switch (h)
  {
  case Trajectory::HEIGHT::HIGH:
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::HIGH));
    break;
  case Trajectory::HEIGHT::MED:
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::MED));
    break;
  case Trajectory::HEIGHT::GROUND:
    m_trajectory = Trajectory::generate_live_traj(Trajectory::determine_desired_traj(Trajectory::HEIGHT::GROUND));
  }
  Trajectory::init_live_traj(m_trajectory);
}

void Robot::make_test_path()
{
  // frc::Pose2d current_pose = Odometry::getPose();
  frc::Pose2d current_pose(frc::Translation2d(0.0_m, 0.0_m), frc::Rotation2d(Drivetrain::getCCWHeading()));

  auto dx = 8.0_ft;
  auto dy = 0.0_ft;

  auto heading = (frc::Translation2d(dx, dy) - current_pose.Translation()).Angle().Degrees();

  m_trajectory = Trajectory::generate_live_traj(current_pose.X(),
                                                current_pose.Y(),
                                                frc::Rotation2d(heading /*current_pose.Rotation().Degrees()*/),
                                                frc::Rotation2d(current_pose.Rotation().Degrees()),
                                                current_pose.X() + dx,
                                                current_pose.Y() + dy,
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
  m_candle.rainbow_anim();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
