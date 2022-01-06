/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.
*/

#pragma once

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/CANSparkMax.h"

#include "navX/NavX.h"
#include "swerve/src/include/SwerveModule.h"
#include "vectors/VectorDouble.h"
#include "recorder/Recorder.h"
#include "commonauto/AsyncLoop.h"
// #include "commonauto/steps/DriveStatic.h"
// #include "commonauto/steps/WaitSeconds.h"

class SwerveTrain {

    public:
        /**
         * Static method that gets the singleton instance of SwerveTrain
         * 
         * This method returns the only allowed instance of SwerveTrain.
         * SwerveTrain is a singleton pattern.
         */
        static SwerveTrain& GetInstance() {

            static SwerveTrain* instance = new SwerveTrain(
                R_CANIDZionFrontRightDrive,
                R_CANIDZionFrontRightSwerve,
                R_CANIDZionFrontLeftDrive,
                R_CANIDZionFrontLeftSwerve,
                R_CANIDZionRearLeftDrive,
                R_CANIDZionRearLeftSwerve,
                R_CANIDZionRearRightDrive,
                R_CANIDZionRearRightSwerve
            );
            return *instance;
        }

        /**
         * 
         * Sets the drive motor's brake mode.
         * 
         * If true, sets the drives to brake mode (as defaultly constructed),
         * if false, sets them to coast. Persists across calls. This is used in
         * the autonomous to allow the swervetrain to Stop very precisely.
         * 
         * @param brake Whether or not to set the brake
         */
        void SetDriveBrake(const bool &brake);

        /**
         * Sets the swerve motor's brake mode.
         * 
         * Same as above for swerve. This is used to
         * "unlock and lock" all of the swerve wheels for easy manual zeroing,
         * instead of fighting the wheel brake as defaultly constructed.
         * 
         * @param brake Whether or not to set the brake
         */
        void SetSwerveBrake(const bool &brake);

        /**
         * Stops all swerve at drive motors.
         * 
         * Sets all swerve motors and drive motors to 0.
         */
        void Stop();

        /**
         * Sets the zero position of all swerve modules.
         * 
         * Gets the current encoder values of the swerve motors and stores them
         * as privates of the class. These are the values the swerve motors return
         * to when invoking AssumeSwerveZeroPosition().
         * If the passed bool is true, publishes the stored data to the
         * SmartDashboard. This is currently used for returning to and maintaining
         * "straight".
         * 
         * @param verbose Whether or not to print the positions
         */
        void SetZeroPosition(const bool &verbose = false);

        /**
         * Drives the swerves to return to their zero position.
         * 
         * @return Whether or not the serves are at their zero positions
         */
        bool AssumeZeroPosition();

        /**
         * Puts data about the swerve modules on the SmardDashboard.
         */
        void DebugSwerveModules();

        /**
         * Drives the swere modules field oriented.
         * 
         * Fully drives the swerve train on the supplied x, y, and z values. If precise,
         * it scales all values according to a R_ constant and doesn't re-center
         * after maneuvering to allow for slow, incredibly precise positioning by
         * hand in the full range of the controller.
         * 
         * @param rawX The x value to drive the SwerveTrain with
         * @param rawY The y value to drive the SwerveTrain with
         * @param rawZ The z value to drive the SwerveTrain with
         * @param precise Whether or not to drive with increased precision
         * @param relative Whether or not to drive relative to the robot and not the field
         * @param hold Whether or not to hold the current angle
         */
        void Drive(const double &x, const double &y, const double rawZ, const bool &precision, const bool &relative, const bool &hold, const double throttle = 1.0);

        void DriveForward() {

            Drive(0, 1.0, 0, false, false, false, 0.25);
        }

        void DriveBackward() {

            Drive(0, -1, 0, false, false, false, 0.25);
        }

        void DriveLeft() {

            Drive(-1, 0, 0, false, false, false, 0.25);
        }

        void DriveRight() {

            Drive(1, 0, 0, false, false, false, 0.25);
        }

        void DriveSpinClockwise() {

            Drive(0, 0, 1, false, false, false, 0.25);
        }

        void DriveSpinCounterclockwise() {
            
            Drive(0, 0, -1, false, false, false, 0.25);
        }

        void DriveForwardFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {

                DriveForward();
            }
            Stop();
        }

        void DriveBackwardFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {
            
                DriveBackward();
            }
            Stop();
        }

        void DriveLeftFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {
                
                DriveLeft();
            }
            Stop();
        }

        void DriveRightFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {
                
                DriveRight();
            }
            Stop();
        }

        void DriveSpinClockwiseFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {
                
                DriveSpinClockwise();
            }
            Stop();
        }

        void DriveSpinCounterclockwiseFixed() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 2) {
            
                DriveSpinCounterclockwise();
            }
            Stop();
        }

        void AssumeZeroPositionForever() {

            while (true) {

                AssumeZeroPosition();
            }
        }

        void WaitASecond() {

            double start = frc::GetTime();
            while (frc::GetTime() - start < 1);
        }

        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;

        enum ZionDirections {

            kForward, kRight, kBackward, kLeft
        };
    
    private:
        SwerveTrain(
            const int frontRightCANDriveID,
            const int frontRightCANSwerveID,
            const int frontLeftCANDriveID,
            const int frontLeftCANSwerveID,
            const int rearLeftCANDriveID,
            const int rearLeftCANSwerveID,
            const int rearRightCANDriveID,
            const int rearRightCANSwerveID
        );

        SwerveTrain(const SwerveTrain&) = delete;
        SwerveTrain& operator = (const SwerveTrain&) = delete;
        SwerveTrain(SwerveTrain&&) = delete;
        SwerveTrain& operator = (SwerveTrain&&) = delete;

        /**
         * Sets a speed to the driving motors on the train. Defaults to zero.
         * 
         * @param driveSpeed The speed to set
         */
        void SetDriveSpeed(const double &driveSpeed);

        /**
         * Sets a speed to all swerve motors on the train. Defaults to zero.
         * 
         * @param swerveSpeed The speed to set
         */
        void SetSwerveSpeed(const double &swerveSpeed);

        /**
         * Drives the swerves to thier turn around center positions.
         * 
         * The swerve turn around center positions are at 45 degree angles in a
         * way that allows the robot to turn around the center as efficiently
         * as possible.
         * 
         * @return Whether or not the swerves are at their turn around center positions
         */
        bool AssumeTurnAroundCenterPositions();

        /**
         * Sets the swerve modules to a vector.
         * 
         * The vector is absolute, or in other words in relation to the field.
         *
         * @param vectorToSet The vector to set the swerve modules to
         * @return Whether or not the swerve modules are at the vector
         */
        bool SetZionMotorsToVector(VectorDouble &vectorToSet);
};