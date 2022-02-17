#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "swerve/src/include/SwerveTrain.h"

class TimeDriveForwardHold : public AutoStep {

    public:
        TimeDriveForwardHold(const double timeToDrive) : AutoStep("TimeDriveForwardHold") {

            m_totalTimeToDrive = timeToDrive;
        }

        void Init() {

            m_startTime = frc::GetTime().value();
        }

        bool Execute() {

            double delta = frc::GetTime().value() - m_startTime;
            double speed = (pow(5, m_totalTimeToDrive - delta) - 1) / (5 - 1);
            if (speed > 1) {

                speed = 1;
            }
            if (delta < m_totalTimeToDrive) {

                SwerveTrain::GetInstance().Drive(0, 1, 0, false, true, true, speed);
                return false;
            }
            else {

                SwerveTrain::GetInstance().Stop();
                return true;
            }
        }

    private:
        double m_totalTimeToDrive;
        double m_startTime;
};