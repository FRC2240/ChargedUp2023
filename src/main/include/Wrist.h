#include <iostream>
#include <rev/CANSparkMax.h>
#include <frc/Solenoid.h>
#include <frc/Compressor.h>
#include "Constants.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <rev/AbsoluteEncoder.h>


class Wrist {

public:

        Wrist();
        ~Wrist();
        void WristPIDInit();
        void WristDashInit();
        void WristDashRead();
        void InitializeEncoders();
        void ReadEncoders();
        void Up();
        void Down();
        void Stop();
        void Follow(double arm_pos);
        void Follow_Flare(double arm_pos);
        void Test();
        
        rev::SparkMaxPIDController m_wrist_PIDController = m_wrist_motor.GetPIDController();

private:
        
       rev::CANSparkMax m_wrist_motor{CONSTANTS::WRIST::WRIST_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};


       struct pidCoeff
       {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
        };

        pidCoeff m_wrist_coeff{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        rev::SparkMaxAbsoluteEncoder m_wrist_Encoder = m_wrist_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
};
