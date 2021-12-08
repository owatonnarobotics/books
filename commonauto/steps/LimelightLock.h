#ifndef LIMELIGHTLOCK_H
#define LIMELIGHTLOCK_H

#include "commonauto/AutoStep.h"
#include "swerve/src/include/SwerveTrain.h"
#include "limelight/Limelight.h"

class LimelightLock : public AutoStep {

    public:
        LimelightLock() : AutoStep("LimelightLock") {}

        void Init() {}

        bool Execute() {

            SwerveTrain::GetInstance().Drive(0, 0, Limelight::GetInstance().CalculateLimelightLockSpeed(), false, false, false);
            return Limelight::GetInstance().isWithinHorizontalTolerance();
        }
};

#endif