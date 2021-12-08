#pragma once

//This is the speed for automatic lateral movement in autonomous.
const double R_zionAutoMovementSpeedLateral = .35;
//And for rotational movement.
const double R_zionAutoMovementSpeedRotational = .2;
//This is the tolerance for autonomous angle assumption in degrees.
const double R_zionAutoToleranceAngle = 10;
//This is how close to zero the Limelight's horizontal target offset can be
//in order to be considered centered.
const double R_zionAutoToleranceHorizontalOffset = .2;

const double R_swerveTrainLimelightLockTolerance = 1;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorAt = 5.0;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorSpeed = .05;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorAt = 2.5;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorSpeed = .025;