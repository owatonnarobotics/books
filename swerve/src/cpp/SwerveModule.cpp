#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "swerve/src/include/SwerveModule.h"

SwerveModule::SwerveModule(const int &canDriveID, const int &canSwerveID) {

    m_driveMotor = new rev::CANSparkMax(canDriveID, rev::CANSparkMax::MotorType::kBrushless);
    m_driveMotorEncoder = new rev::CANEncoder(m_driveMotor->GetEncoder());
    m_swerveMotor = new rev::CANSparkMax(canSwerveID, rev::CANSparkMax::MotorType::kBrushless);
    m_swerveMotorEncoder = new rev::CANEncoder(m_swerveMotor->GetEncoder());

    //Default the swerve's zero position to its power-on position.
    m_swerveZeroPosition = m_swerveMotorEncoder->GetPosition();
    m_lastSwerveSpeedSet = 0;

    //Allow the drive motor to coast, but brake the swerve motor for accuracy.
    //These must be set as they become overwritten from code.
    m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    Stop();
}

void SwerveModule::SetDriveSpeed(const double &speedToSet) {

    m_driveMotor->Set(speedToSet);
}

void SwerveModule::SetSwerveSpeed(const double &speedToSet) {

    m_swerveMotor->Set(speedToSet);
    m_lastSwerveSpeedSet = speedToSet;
}

void SwerveModule::SetDriveBrake(const bool &brake) {

    if (brake) {

        m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else {

        m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

void SwerveModule::SetSwerveBrake(const bool &brake) {

    if (brake) {

        m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
    else {

        m_swerveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
}

void SwerveModule::Stop() {

    SetDriveSpeed(0);
    SetSwerveSpeed(0);
}

void SwerveModule::SetZeroPosition() {

    m_swerveZeroPosition = m_swerveMotorEncoder->GetPosition();
}

double SwerveModule::GetDrivePosition() {

    return m_driveMotorEncoder->GetPosition();
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

    return m_driveMotorEncoder->GetVelocity();
}

double SwerveModule::GetSwerveSpeed() {

    return m_swerveMotorEncoder->GetVelocity();
}

double SwerveModule::AbsoluteVectorToNics(VectorDouble &vector, const double &angle) {

    return R_nicsConstant * (vector.unitCircleAngleDeg() + angle - 90.0) / 360.0;
}

bool SwerveModule::AssumeSwervePosition(const double &positionToAssumeRaw, bool log) {

    double toCalculate = 0.0;

    double modded = SingleNic(positionToAssumeRaw);
    double delta = modded - GetSwervePosition();
    if (abs(delta) > R_nicsConstant / 2.0) {

        if (delta > 0) {

            toCalculate = -(R_nicsConstant - delta);
        }
        else {

            toCalculate = R_nicsConstant - (-delta);
        }
    }
    else {

        toCalculate = delta;
    }

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

    frc::SmartDashboard::PutNumber(header + "_RawPosition", m_swerveMotorEncoder->GetPosition());
    frc::SmartDashboard::PutNumber(header + "_ZeroPosition", m_swerveZeroPosition);
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
