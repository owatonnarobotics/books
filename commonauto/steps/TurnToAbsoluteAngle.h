#pragma once

#include "commonauto/AutoStep.h"
#include "StepConsts.h"
#include "navX/NavX.h"
#include "swerve/src/include/SwerveTrain.h"
#include "geo/GeoUtils.h"

class TurnToAbsoluteAngle : public AutoStep {

    public:
        TurnToAbsoluteAngle(const double angle) : AutoStep("TurnToAbsoluteAngle") {

            m_targetAngle = angle;
        }

        void Init() {}

        bool Execute() {

            double toCalculate = GeoUtils::MinDistFromDelta(m_targetAngle - NavX::GetInstance().getYawFull(), 360);

            if (abs(toCalculate) > TURN_TO_ANGLE_ABSOLUTE__TOLERANCE) {

                double speed = (pow(5, 1 / 90.0 * abs(toCalculate)) - 1) / (5 - 1);
                if (speed < 0.25) {

                    speed = 0.1;
                }
                if (toCalculate < 0) {

                    speed *= -1;
                }
                if (speed > 1) {

                    speed = 1;
                }
                else if (speed < -1) {

                    speed = -1;
                }
                SwerveTrain::GetInstance().Drive(0, 0, speed, false, false, AUTO_EXECUTION_CAP);
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