#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "swerve/src/include/SwerveModule.h"

SwerveModule::SwerveModule(const int &canDriveID, const int &canSwerveID, const int &canEncoderID) {

    m_driveMotor = new ctre::phoenix::motorcontrol::can::TalonFX(canDriveID);
    // TODO: This is 13 for all SwerveModules because no CANCoders are
    // installed on the drive motors.
    m_driveMotorEncoder = nullptr;//new ctre::phoenix::sensors::CANCoder(13);
    m_swerveMotor = new ctre::phoenix::motorcontrol::can::TalonFX(canSwerveID);
    m_swerveMotorEncoder = new ctre::phoenix::sensors::CANCoder(canEncoderID);

    // m_driveMotorEncoder->ConfigFeedbackCoefficient(R_CANCoderMaxDefaultValue / R_nicsConstant, "spmxs", ctre::phoenix::sensors::SensorTimeBase::PerSecond);
    m_swerveMotorEncoder->ConfigFeedbackCoefficient(R_nicsConstant / R_CANCoderMaxDefaultValue, "spmxs", ctre::phoenix::sensors::SensorTimeBase::PerSecond);

    //Default the swerve's zero position to its power-on position.
    m_swerveZeroPosition = m_swerveMotorEncoder->GetPosition();
    m_lastSwerveSpeedSet = 0;

    Stop();
}

void SwerveModule::SetDriveSpeed(const double &speedToSet) {

    m_driveMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
}

void SwerveModule::SetSwerveSpeed(const double &speedToSet) {

    m_swerveMotor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speedToSet);
    m_lastSwerveSpeedSet = speedToSet;
}

void SwerveModule::Stop() {

    SetDriveSpeed(0);
    SetSwerveSpeed(0);
}

void SwerveModule::HardwareZero() {

    m_swerveMotorEncoder->SetPosition(0);
    m_swerveZeroPosition = 0;
}

double SwerveModule::GetDrivePosition() {

    return 0;//m_driveMotorEncoder->GetPosition();
}

// Initially, this function would return the raw encoder value. Now, it returns
// the relative position of the swerve.
double SwerveModule::GetSwervePosition() {

    return SingleNic(m_swerveMotorEncoder->GetPosition() - m_swerveZeroPosition);
}

// Maps any multiple of a nic to a nic in the range of 0-nic. If the value is 
double SwerveModule::SingleNic(const double nics) {

    return fmod(nics, R_nicsConstant) + (nics < 0 ? R_nicsConstant : 0);
}

double SwerveModule::GetDriveSpeed() {

    return 0;//m_driveMotorEncoder->GetVelocity();
}

double SwerveModule::GetSwerveSpeed() {

    return m_swerveMotorEncoder->GetVelocity();
}

double SwerveModule::AbsoluteVectorToNics(VectorDouble &vector, const double &angle) {

    return R_nicsConstant * (vector.unitCircleAngleDeg() + angle - 90.0) / 360.0;
}

bool SwerveModule::AssumeSwervePosition(const double &positionToAssumeRaw, bool log) {

    double modded = SingleNic(positionToAssumeRaw);
    double delta = modded - GetSwervePosition();

    double toCalculate = GeoUtils::MinDistFromDelta(delta, R_nicsConstant);

    if (abs(toCalculate) > R_swerveTrainAssumePositionTolerance) {

        SetSwerveSpeed(calculateAssumePositionSpeed(toCalculate));
        return false;
    }
    else {

        SetSwerveSpeed(0);
        return true;
    }
}

bool SwerveModule::AssumeSwerveZeroPosition() {

    return AssumeSwervePosition(0);
}

void SwerveModule::Debug(std::string header) {

    frc::SmartDashboard::PutNumber(header + "_GetPosition", m_swerveMotorEncoder->GetPosition());
    frc::SmartDashboard::PutNumber(header + "_OffsetFromZeroPosition", m_swerveMotorEncoder->GetPosition() - m_swerveZeroPosition);
    frc::SmartDashboard::PutNumber(header + "_ZeroPosition", m_swerveZeroPosition);
    frc::SmartDashboard::PutNumber(header + "_GetAbsolutePosition", m_swerveMotorEncoder->GetAbsolutePosition());
    frc::SmartDashboard::PutNumber(header + "_RelativePosition", GetSwervePosition());
    frc::SmartDashboard::PutNumber(header + "_LastSpeedSet", m_lastSwerveSpeedSet);
}

double SwerveModule::calculateAssumePositionSpeed(const double &howFarRemainingInTravel) {

    //Begin initally with a double calculated with the simplex function...
    double toReturn = ((1) / (1 + exp((-1 * abs(howFarRemainingInTravel)) + 5)));
    //If we satisfy conditions for the first linear piecewiseposition, take that speed instead...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationFirstEndBehaviorSpeed;
    }
    //Do the same for the second...
    if (abs(howFarRemainingInTravel) < R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorAt) {

        toReturn = R_swerveTrainAssumePositionSpeedCalculationSecondEndBehaviorSpeed;
    }
    //And if we needed to travel negatively to get where we need to be, make the final speed negative...
    if (howFarRemainingInTravel < 0) {

        toReturn = -toReturn;
    }
    return toReturn;
}
