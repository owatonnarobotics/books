#pragma once

//The amount of REV rotations it takes for a swerve assembly to make a full rotation.
//Often, a REV Rotation is referred to as a Nic, although they mean different things.
//Truly, a Nic is ~17.976 REV Rotation values.
const double R_nicsConstant = 17.9761447906494;
//The change in encoder output per full wheel rotation around the axle.
//This value can be used to move a certain distance using solely encoder values.
const double R_kuhnsConstant = 8.312033026;
//If an xy coordinate plane is centered at the middle of the drivetrain, this
//is the radian measure between the y-axis and the front right wheel. This is
//the basic unit of a non-moving center turn, and it is modified as the basis
//for moving and turning at the same time.
const double R_angleFromCenterToFrontLeftWheel = 45.;
const double R_angleFromCenterToFrontRightWheel = 315.;
const double R_angleFromCenterToRearLeftWheel = 135.;
const double R_angleFromCenterToRearRightWheel = 225.;

//These contants are used for the functions which provide assuming a position.
//See those functions for further detail.
const double R_swerveTrainAssumePositionTolerance = .25;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .02;

const double R_swerveTrainLimelightLockTolerance = 1;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorAt = 5.0;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonFirstEndBehaviorSpeed = .05;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorAt = 2.5;
const double R_swerveTrainLimelightLockPositionSpeedCalculatonSecondEndBehaviorSpeed = .025;

const double R_swerveTrainHoldAngleTolerance = 1.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorAt = 10.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorSpeed = .075;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorAt = 5.0;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorSpeed = .075;