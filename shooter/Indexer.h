#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/VictorSP.h>
#include "ShooterConsts.h"

    class Indexer{
        public:
         static Indexer& GetInstance() {
            static Indexer* instance = new Indexer(R_IndexerCANID);
            return *instance;
        }
        
        void SetIndexerSpeed(const double &speedToSet) {
            m_indexerMotor->Set(speedToSet);
        }
        private:
         Indexer(const int R_IndexerCANID) {
           
            m_indexerMotor = new frc::VictorSP(R_IndexerCANID);
        }
        
        frc::VictorSP *m_indexerMotor;

    };