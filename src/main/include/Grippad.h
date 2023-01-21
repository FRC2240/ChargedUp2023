#include <frc/Compressor.h>
#include <frc/Solenoid.h>

class Grippad
{

 public:

         Grippad();

         ~Grippad();

         void Deployed();

         void Stored();


 private:
        //Grippad has four pistons
         frc::Solenoid m_grippad_piston{frc::PneumaticsModuleType::CTREPCM,0};
         frc::Solenoid m_grippad_piston{frc::PneumaticsModuleType::CTREPCM,0};
         frc::Solenoid m_grippad_piston{frc::PneumaticsModuleType::CTREPCM,0};
         frc::Solenoid m_grippad_piston{frc::PneumaticsModuleType::CTREPCM,0};




}
