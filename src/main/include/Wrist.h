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
        void pickupFollow(double arm_pos);
        void Follow(double arm_pos);
        void test();
        
        rev::SparkMaxPIDController m_wrist_PIDController = m_wrist_motor.GetPIDController();

private:
        
       rev::CANSparkMax m_wrist_motor{CONSTANTS::WRIST::WRIST_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless};

        double position = 0.0;

       struct pidCoeff
       {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
        };

        pidCoeff m_wrist_coeff{2.2, 0.0, 8.0, 0.0, 0.0, 1.0, -1.0};
        rev::SparkMaxAbsoluteEncoder m_wrist_Encoder = m_wrist_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
};
