#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"

class TurnToAbsoluteAngle : public AutoStep {

    public:
        TurnToAbsoluteAngle(const double angle) : AutoStep("TurnToAbsoluteAngle") {

            m_targetAngle = angle;
        }

        void Init() {}

        bool Execute() {

            double toCalculate = 0.0;

            double delta = m_targetAngle - NavX::GetInstance().getYawFull();
            if (abs(delta) > 180) {

                if (delta > 0) {

                    toCalculate = -(360 - delta);
                }
                else {

                    toCalculate = 360 - (-delta);
                }
            }
            else {

                toCalculate = delta;
            }

            if (abs(toCalculate) > TURN_TO_ANGLE_ABSOLUTE__TOLERANCE) {

                double speed = (pow(5, 1 / 10.0 * abs(toCalculate)) - 1) / (5 - 1);
                if (toCalculate < 0) {

                    speed *= -1;
                }
                if (speed > 1) {

                    speed = 1;
                }
                else if (speed < -1) {

                    speed = -1;
                }
                SwerveTrain::GetInstance().Drive(0, 0, speed, false, false, false, AUTO_EXECUTION_CAP);
                return false;
            }
            else {

                SwerveTrain::GetInstance().Stop();
                return true;
            }
        }        

    private:
        double m_targetAngle;
};