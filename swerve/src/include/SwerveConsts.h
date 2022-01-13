#pragma once

/*_____RoboRIO CAN Bus ID Declarations_____*/
const int R_CANIDZionFrontRightSwerve = 7;
const int R_CANIDZionFrontRightDrive  = 8;
const int R_CANIDZionFrontLeftSwerve  = 1;
const int R_CANIDZionFrontLeftDrive   = 2;
const int R_CANIDZionRearLeftSwerve   = 3;
const int R_CANIDZionRearLeftDrive    = 4;
const int R_CANIDZionRearRightSwerve  = 5;
const int R_CANIDZionRearRightDrive   = 6;

const int R_CANIDZionFrontRightEncoder = 12;
const int R_CANIDZionFrontLeftEncoder = 9;
const int R_CANIDZionRearLeftEncoder = 10;
const int R_CANIDZionRearRightEncoder = 11;
/*___End RoboRIO CAN Bus ID Declarations___*/

// The CANCoder's default encoder max 12-bit value
const int R_CANCoderMaxDefaultValue = 4096;
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
const double R_angleFromCenterToFrontLeftWheel = 45.0;
const double R_angleFromCenterToFrontRightWheel = 315.;
const double R_angleFromCenterToRearLeftWheel = 135.;
const double R_angleFromCenterToRearRightWheel = 225.;

//These contants are used for the functions which provide assuming a position.
//See those functions for further detail.
const double R_swerveTrainAssumePositionTolerance = .25;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt = 3.5;
const double R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed = .2;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt = 1;
const double R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed = .10;

const double R_swerveTrainHoldAngleTolerance = 1.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorAt = 10.0;
const double R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorSpeed = .075;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorAt = 5.0;
const double R_swerveTrainHoldAngleSpeedCalculatonSecondEndBehaviorSpeed = .075;