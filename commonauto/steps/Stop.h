#pragma once

#include "commonauto/AutoStep.h"

#include "swerve/src/include/SwerveTrain.h"

class Stop : public AutoStep {

    public:
        Stop() : AutoStep("Stop") {}

        void Init() {}

        bool Execute() {

            SwerveTrain::GetInstance().Stop();
            return true;
        }
};