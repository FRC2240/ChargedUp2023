#include <iostream>
#include <rev/CANSparkMax.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Compressor.h>
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"


class Grabber {

public:

        Grabber();
        ~Grabber();
        void WristPIDInit();
        void WristDashInit();
        void WristDashRead();
        
        rev::SparkMaxPIDController m_wrist_PIDController = m_wrist_motor.GetPIDController();

private:

        frc::DoubleSolenoid m_grabberPiston{frc::PneumaticsModuleType::CTREPCM,0, 7};
        
       rev::CANSparkMax m_wrist_motor{CONSTANTS::GRABBER::WRIST_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};


       struct pidCoeff
       {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
        };

        pidCoeff m_wrist_coeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

};