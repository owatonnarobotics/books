#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/VictorSP.h>
#include "ShooterConsts.h"

// #include "books/shooter/ShooterConsts.h"

class Shooter {
    
    public:
        static Shooter& GetInstance() {
            static Shooter* instance = new Shooter(R_ShooterCANID, R_IndexerCANID);
            return *instance;
        }

        void SetShooterSpeed(const double &speedToSet) {
            m_shooterMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
        }

        void SetIndexerSpeed(const double &speedToSet) {
            m_indexerMotor->Set(speedToSet);
        }

    private:
        Shooter(const int R_ShooterCANID, const int R_IndexerCANID) {
            m_shooterMotor = new ctre::phoenix::motorcontrol::can::TalonFX(R_ShooterCANID);
            m_indexerMotor = new frc::VictorSP(R_IndexerCANID);
        }
        ctre::phoenix::motorcontrol::can::TalonFX *m_shooterMotor; //joe's contribution
        frc::VictorSP *m_indexerMotor;

};