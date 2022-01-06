/*
class SwerveModule
*/

#pragma once

#include <math.h>

#include "rev/CANSparkMax.h"

#include "swerve/src/include/SwerveConsts.h"
#include "vectors/VectorDouble.h"

class SwerveModule {

    public:
        /**
         * Constructor for a SwerveModule.
         * 
         * Creates a swerve module with Spark MAX motor controllers on the
         * two supplied CAN IDs, the first controlling drive, the second
         * controlling swerve.
         * 
         * @param canDriveID The ID of the drive motor's Spark MAX on the bus
         * @param canSwerveID The ID of the swerve motor's Spark MAX on the bus
         */
        SwerveModule(const int &canDriveID, const int &canSwerveID);

        /**
         * Sets the driving speed to a double. Defaults to zero.
         * 
         * @param speedToSet The speed to set
         */
        void SetDriveSpeed(const double &speedToSet = 0);

        /**
         * Sets the swerve speed to a double. Defaults to zero.
         * 
         * @param speedToSet The speed to set
         */
        void SetSwerveSpeed(const double &speedToSet = 0);

        /**
         * Sets the drive motor's brake mode.
         * 
         * If true, sets the swerve to brake mode, if false, to coast mode.
         * This is used in SwerveTrain to allow "unlocking" the swerve wheels
         * for zeroing by overriding the default brake initialization.
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
         * Stops the swerve and drive motors.
         * 
         * Sets the swerve motor and drive motor to 0.
         */
        void Stop();

        /**
         * Sets the zero position of the swerve module.
         * 
         * Gets the current encoder value of the swerve motor and store it
         * as private of the class. This is the value the swerve motor returns
         * to when invoking AssumeSwerveZeroPosition().
         */
        void SetZeroPosition();

        /**
         * Get the total REV revolutions of the drive encoder.
         * 
         * @return The total REV revolutions of the drive encoder.
         */
        double GetDrivePosition();

        /**
         * Get the total REV revolutions of the swerve encoder.
         * 
         * @return The total REV revolutions of the swerve encoder.
         */
        double GetSwervePosition();

        /**
         * Converts a +/- swerve position to a range of 0 to Nic's Constant.
         * 
         * Example: -R_nicsConstant / 4 -> R_nicsConstant * 3 / 4.
         *
         * @param nics The value to convert
         * @return The position in a range from 0 to Nic's Constant
         */
        double SingleNic(const double nics);

        /**
         * Returns the zero position of the swerve encoder (whatever the value of
         * its variable is).
         * 
         * @return The zero position
         */
        double GetSwerveZeroPosition();

        /**
         * Returns the speed of the drive encoder in RPM.
         * 
         * @return The speed of the drive encoder in RPM
         */
        double GetDriveSpeed();

        /**
         * Returns the speed of the swerve encoder in RPM.
         * 
         * @return The speed of the swerve encoder in RPM
         */
        double GetSwerveSpeed();

        /**
         * Convert the direction of a vector to nics
         * 
         * @param vector The vector to convert
         * @return The amount of nics
         */
        double AbsoluteVectorToNics(VectorDouble &vector, const double &angle);

        /**
         * Set the swerve module to a position.
         * 
         * Uses a mathematical function to assign a speed to the swerve motor to
         * move quickly and accurately, within a tolerance, to any REV rotation
         * value, clockwise or counterclockwise, with an optimal path. See the
         * function itself for further detail.
         *
         * @param positionToAssume The position to set the swerve module to
         * @return Whether or not the swerve module is at the position
         */
        bool AssumeSwervePosition(const double &positionToAssumeRaw, bool log = false);

        /**
         * Drives the swerve to the current value of the swerve's zero position
         * variable (the last set zero position).
         * 
         * @return Whether or not the swerve modules is at its zero position
         */
        bool AssumeSwerveZeroPosition();

        /**
         * @brief Publishes data to the SmartDashboard about the swerve module
         * 
         * Publishes many different points of data to the SmartDashboard at the
         * same time; this is so that if other modules are debugged as well,
         * each module's data will be grouped.
         * 
         * @param header the string to print before each data poin
         */
        void Debug(std::string header);

    private:
        /**
         * Uses the following function
         *      {(1)/(1+e^((-1 * abs(z)) + 5)); z >= R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
         * s(z)={R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt
         *      {R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed; z < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt
         *     for
         *         s = speed at which the motor rotates to assume a position
         *         z = remaining REV revolutions of the position assumption
         * to assign a speed with which to proceed towards the final position. It
         * was developed, regressed, and tuned to move to the final position as
         * fast as possible initially, slowing down as it approaches and becoming
         * linear as it settles into tolerance at a high accuracy.
         * 
         * @param howFarRemainingInTravel The remaining nics
         * @return The speed
         */
        double calculateAssumePositionSpeed(const double &howFarRemainingInTravel);

        rev::CANSparkMax *m_driveMotor;
        rev::CANEncoder *m_driveMotorEncoder;
        rev::CANSparkMax *m_swerveMotor;
        rev::CANEncoder *m_swerveMotorEncoder;

        double m_swerveZeroPosition;
        double m_lastSwerveSpeedSet;
};
