/*
class SwerveTrain

    Allows higher-level control of four SwerveModules as a drivetrain, and
        provides many private controller functions for manipulating it.
*/

#pragma once

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "ctre/Phoenix.h"

#include "navX/NavX.h"
#include "swerve/src/include/SwerveModule.h"
#include "vectors/VectorDouble.h"
#include "recorder/Recorder.h"
#include "commonauto/AsyncLoop.h"
#include "logging/Logger.h"
#include "geo/GeoUtils.h"

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
                R_CANIDZionFrontRightEncoder,
                R_CANIDZionFrontLeftDrive,
                R_CANIDZionFrontLeftSwerve,
                R_CANIDZionFrontLeftEncoder,
                R_CANIDZionRearLeftDrive,
                R_CANIDZionRearLeftSwerve,
                R_CANIDZionRearLeftEncoder,
                R_CANIDZionRearRightDrive,
                R_CANIDZionRearRightSwerve,
                R_CANIDZionRearRightEncoder
            );
            return *instance;
        }

        /**
         *
         * Set the all swerve motor neutral modes
         * 
         * @param brake whether or not to set the brake
         */
        void SetSwerveBrake(const bool brake) {

            m_frontRight->SetSwerveBrake(brake);
            m_frontLeft->SetSwerveBrake(brake);
            m_rearLeft->SetSwerveBrake(brake);
            m_rearRight->SetSwerveBrake(brake);
        }

        /**
         *
         * Set the all drive motor neutral modes
         * 
         * @param brake whether or not to set the brake
         */
        void SetDriveBrake(const bool brake) {

            m_frontRight->SetDriveBrake(brake);
            m_frontLeft->SetDriveBrake(brake);
            m_rearLeft->SetDriveBrake(brake);
            m_rearRight->SetDriveBrake(brake);
        }

        /**
         * Stops all swerve at drive motors.
         * 
         * Sets all swerve motors and drive motors to 0.
         */
        void Stop();

        /**
         * Resets each swerve module's swerve encoder.
         * 
         * Resets each swerve module's swerve encoder to output 0 as the
         * current position.
         */
        void HardwareZero();

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
         * @param relative Whether or not to drive relative to the robot and not the field
         * @param hold Whether or not to hold the current angle
         */
        void Drive(const double &x, const double &y, double z, const bool relative, const bool hold, const double throttle = 1.0);

        SwerveModule *m_frontRight;
        SwerveModule *m_frontLeft;
        SwerveModule *m_rearLeft;
        SwerveModule *m_rearRight;

        enum ZionDirections {

            kForward, kRight, kBackward, kLeft
        };

        void ResetHold() {

            m_holdAngle = 0;
            m_wasHolding = false;
            m_wasRelative = false;
        }

        /**
         * Updates the odometry of the SwerveTrain.
         * 
         * Updates the SwerveTrain's odometry by getting velocity and rotation
         * from each swerve module.
         */
        void UpdateOdometry();
    
    private:
        SwerveTrain(
            const int frontRightCANDriveID,
            const int frontRightCANSwerveID,
            const int frontRightCANEncoderID,
            const int frontLeftCANDriveID,
            const int frontLeftCANSwerveID,
            const int frontLeftCANEncoderID,
            const int rearLeftCANDriveID,
            const int rearLeftCANSwerveID,
            const int rearLeftCANEncoderID,
            const int rearRightCANDriveID,
            const int rearRightCANSwerveID,
            const int rearRightCANEncoderID
        );

        frc::SwerveDriveOdometry<4>* m_odometry;

        SwerveTrain(const SwerveTrain&) = delete;
        SwerveTrain& operator = (const SwerveTrain&) = delete;
        SwerveTrain(SwerveTrain&&) = delete;
        SwerveTrain& operator = (SwerveTrain&&) = delete;

        bool m_wasHolding;
        bool m_wasRelative;
        double m_holdAngle;

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

        /**
         * Logs a string to stdout (riolog).
         * 
         * Logs a string to stdout (riolog).
         * 
         * @param msg The message to log
         */
        void Log(std::string msg);
};