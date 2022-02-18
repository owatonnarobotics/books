#pragma once

#include "commonauto/AutoStep.h"

#include "navX/NavX.h"

class ResetNavXYaw : public AutoStep {

    public:
        ResetNavXYaw() : AutoStep("ResetNavXYaw") {}

        void Init() {}

        bool Execute() {

            NavX::GetInstance().resetYaw();
            return true;
        }
};