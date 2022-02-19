#pragma once

#include "commonauto/AutoStep.h"

#include "navX/NavX.h"

class CalibrateNavXThenReset : public AutoStep {

    public:
        CalibrateNavXThenReset() : AutoStep("CalibrateNavXThenReset") {}

        void Init() {

            NavX::GetInstance().Calibrate();
        }

        bool Execute() {

            if (!NavX::GetInstance().isCalibrating()) {

                NavX::GetInstance().resetYaw();
                return true;
            }
            else {

                return false;
            }
        }
};