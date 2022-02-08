#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/motorcontrol/VictorSP.h>
#include "Shooter/ShooterConsts.h"
class Intake {

public:

    static Intake& GetInstance() {
            static Intake* instance = new Intake(R_IntakeCANID);
            return *instance;
        }
    void SetIntakeSpeed(const double &speedToSet) {
            m_intakeMotor->Set(speedToSet);
    }
private:
         Intake(const int R_IntakeCANID) {
           
            m_intakeMotor = new frc::VictorSP(R_IntakeCANID);
        }
        
        frc::VictorSP *m_intakeMotor;
};
