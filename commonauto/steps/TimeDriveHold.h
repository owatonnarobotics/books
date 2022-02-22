#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "swerve/src/include/SwerveTrain.h"

class TimeDriveHold : public AutoStep {

    public:
        TimeDriveHold(const double x, const double y, const double timeToDrive, const bool jerk = false) : AutoStep("TimeDriveHold") {

            m_totalTimeToDrive = timeToDrive;
            m_x = x;
            m_y = y;
            m_jerk = jerk;
        }

        void Init() {

            m_startTime = frc::GetTime().value();
            std::cout << NavX::GetInstance().getYawFull() << std::endl;
        }

        bool Execute() {

            double speed = 0;
            double delta = frc::GetTime().value() - m_startTime;
            if (!m_jerk) {
                
                if (delta <= m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, delta) - 1) / (double)(5 - 1);
                }
                else if (delta > m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, m_totalTimeToDrive - delta) - 1) / (double)(5 - 1);
                }
                if (speed > 1) {

                    speed = 1;
                }
            }
            else {

                if (delta > m_totalTimeToDrive / 2.0) {

                    speed = (pow(5, m_totalTimeToDrive - delta) - 1) / (double)(5 - 1);
                }
                else {
                    
                    speed = 1;
                }
            }
            if (delta < m_totalTimeToDrive) {

                SwerveTrain::GetInstance().Drive(m_x, m_y, 0, false, true, speed * AUTO_EXECUTION_CAP);
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
        double m_x;
        double m_y;
        bool m_jerk;
};