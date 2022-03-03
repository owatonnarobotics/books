#include <math.h>

#include <frc/DriverStation.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "swerve/src/include/SwerveTrain.h"

SwerveTrain::SwerveTrain(
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
    ) {

    m_frontRight = new SwerveModule(frontRightCANDriveID, frontRightCANSwerveID, frontRightCANEncoderID);
    m_frontLeft = new SwerveModule(frontLeftCANDriveID, frontLeftCANSwerveID, frontLeftCANEncoderID);
    m_rearLeft = new SwerveModule(rearLeftCANDriveID, rearLeftCANSwerveID, rearLeftCANEncoderID);
    m_rearRight = new SwerveModule(rearRightCANDriveID, rearRightCANSwerveID, rearRightCANEncoderID);

    frc::Translation2d frontLeftLocation(R_distanceFromCenterToSwerveModuleLateral, R_distanceFromCenterToSwerveModuleLateral);
    frc::Translation2d frontRightLocation(-R_distanceFromCenterToSwerveModuleLateral, R_distanceFromCenterToSwerveModuleLateral);
    frc::Translation2d backLeftLocation(R_distanceFromCenterToSwerveModuleLateral, -R_distanceFromCenterToSwerveModuleLateral);
    frc::Translation2d backRightLocation(-R_distanceFromCenterToSwerveModuleLateral, -R_distanceFromCenterToSwerveModuleLateral);

    frc::SwerveDriveKinematics<4> kinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    m_odometry = new frc::SwerveDriveOdometry<4>(kinematics, units::degree_t(NavX::GetInstance().getYawFull()), frc::Pose2d(0_m, 0_m, 0_rad));

    m_wasHolding = false;
    m_wasRelative = false;
    m_holdAngle = 0;
}

void SwerveTrain::SetDriveSpeed(const double &driveSpeed) {

    m_frontRight->SetDriveSpeed(driveSpeed);
    m_frontLeft->SetDriveSpeed(driveSpeed);
    m_rearLeft->SetDriveSpeed(driveSpeed);
    m_rearRight->SetDriveSpeed(driveSpeed);
}

void SwerveTrain::SetSwerveSpeed(const double &swerveSpeed) {

    m_frontRight->SetSwerveSpeed(swerveSpeed);
    m_frontLeft->SetSwerveSpeed(swerveSpeed);
    m_rearLeft->SetSwerveSpeed(swerveSpeed);
    m_rearRight->SetSwerveSpeed(swerveSpeed);
}

void SwerveTrain::Stop() {

    SetDriveSpeed(0);
    SetSwerveSpeed(0);
}

void SwerveTrain::HardwareZero() {

    m_frontRight->HardwareZero();
    m_frontLeft->HardwareZero();
    m_rearLeft->HardwareZero();
    m_rearRight->HardwareZero();
    Log("Hardware zeroed");
}

bool SwerveTrain::AssumeZeroPosition() {

    bool fr = m_frontRight->AssumeSwerveZeroPosition();
    bool fl = m_frontLeft->AssumeSwerveZeroPosition();
    bool rl = m_rearLeft->AssumeSwerveZeroPosition();
    bool rr = m_rearRight->AssumeSwerveZeroPosition();
    return fr && fl && rl && rr;
}

bool SwerveTrain::AssumeTurnAroundCenterPositions() {

    bool fr = m_frontRight->AssumeSwervePosition((1.0 / 8.0) * R_nicsConstant);
    bool fl = m_frontLeft->AssumeSwervePosition((3.0 / 8.0) * R_nicsConstant);
    bool rl = m_rearLeft->AssumeSwervePosition((5.0 / 8.0) * R_nicsConstant);
    bool rr = m_rearRight->AssumeSwervePosition((7.0 / 8.0) * R_nicsConstant);
    return  fr && fl && rl && rr;
}

bool SwerveTrain::SetZionMotorsToVector(VectorDouble &vectorToSet) {

    double angle = NavX::GetInstance().getYawFull();
    bool fr = m_frontRight->AssumeSwervePosition(m_frontRight->AbsoluteVectorToNics(vectorToSet, angle));
    bool fl = m_frontLeft->AssumeSwervePosition(m_frontLeft->AbsoluteVectorToNics(vectorToSet, angle));
    bool rl = m_rearLeft->AssumeSwervePosition(m_rearLeft->AbsoluteVectorToNics(vectorToSet, angle));
    bool rr = m_rearRight->AssumeSwervePosition(m_rearRight->AbsoluteVectorToNics(vectorToSet, angle));
    return  fr && fl && rl && rr;
}

void SwerveTrain::DebugSwerveModules() {

    m_frontRight->Debug("FR");
    m_frontLeft->Debug("FL");
    m_rearLeft->Debug("RL");
    m_rearRight->Debug("RR");
}

void SwerveTrain::UpdateOdometry() {

    m_odometry->Update(
        units::degree_t(-NavX::GetInstance().getYawFull()),
        m_frontLeft->GetState(),
        m_frontRight->GetState(),
        m_rearLeft->GetState(),
        m_rearRight->GetState()
    );

    // frc::Pose2d currentPose = m_odometry->GetPose();
    // frc::SmartDashboard::PutNumber("X", currentPose.X().value());
    // frc::SmartDashboard::PutNumber("Y", currentPose.Y().value());
    // frc::SmartDashboard::PutNumber("Rotation", currentPose.Rotation().Degrees().value());
}

void SwerveTrain::Drive(const double &x, const double &y, double z, const bool relative, const bool hold, const double throttle) {

    if (!hold && x == 0 && y == 0 && z == 0) {

        Stop();
    }
    else {

        double angle = NavX::GetInstance().getYawFull();

        if (hold) {

            if (!m_wasHolding) {
            
                m_holdAngle = angle;
            }

            double toCalculate = GeoUtils::MinDistFromDelta(angle - m_holdAngle, 360);
            //Update our rotational speed so that we turn towards the goal.
            //Begin initally with a double calculated with the simplex function with a horizontal stretch of factor two...
            z = ((1) / (1 + exp((-1 * abs(1.0 / 4.0 * toCalculate)) + 5)));
            //If we satisfy conditions for the first linear piecewise, take that speed instead...
            if (abs(toCalculate) < R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorAt) {

                z = R_swerveTrainHoldAngleSpeedCalculatonFirstEndBehaviorSpeed;
            }
            //And if we needed to travel negatively to get where we need to be, make the final speed negative...
            if (abs(toCalculate) < R_swerveTrainHoldAngleTolerance) {

                z = 0;
            }
            if (toCalculate < 0) {

                z = -z;
            }
            z *= -1;
        }
        if (relative) {

            angle = 0;        
        }

        m_wasHolding = hold;
        m_wasRelative = relative;

        /*
        The translation vector is the "standard" vector - that is, if no
        rotation were applied, the robot would simply travel in the direction
        of this vector. In order to obtain this, we need the X and Y from the
        controller in addition to input from a gyroscope. This is due to the
        fact that pushing straight on the controller should always make it
        drive directly away from the operator, and simply driving "straight" at
        a 45* angle would make it drive away from the operator at 45*. This is
        true of any angle, so the gyro is needed to offset the vector described
        by X and Y. VectorDouble translationVector(0, 0);
        */
        //TODO: why inverted?
        VectorDouble translationVector(-x, y);

        /*
        The rotational vectors are found by multiplying the controller's
        rotational axis [-1, 1] by the cosine of the wheel's RELATIVE yaw (the
        position we put the wheels in so that it can turn, with zero at the
        top) minus the number of degrees we are offset from 0.  Then, for the j
        value we do the same, except we use sine, for Y.  All angles passed as
        paramaters to cos() and sin() are converted to radians first.
        */
        VectorDouble frontRightRotationVector (

            z * cos((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontRightWheel - angle) * (M_PI / 180))
        );

        VectorDouble frontLeftRotationVector (

            z * cos((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToFrontLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearLeftRotationVector (

            z * cos((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearLeftWheel - angle) * (M_PI / 180))
        );

        VectorDouble rearRightRotationVector (

            z * cos((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180)),
            z * sin((R_angleFromCenterToRearRightWheel - angle) * (M_PI / 180))
        );

        /*
        And the vector we actually want to apply to the swerves is the sum of
        the two vectors - the vector that forms "straight" (translationVector)
        and the vector that forms strictly the rotation (rotationVector).
        */
        VectorDouble frontRightResultVector = translationVector + frontRightRotationVector;
        VectorDouble frontLeftResultVector = translationVector + frontLeftRotationVector;
        VectorDouble rearLeftResultVector = translationVector + rearLeftRotationVector;
        VectorDouble rearRightResultVector = translationVector + rearRightRotationVector;

        /*
        Here, all of the resulting vectors are converted into Nics so that they
        can be written to the swerve modules using AssumeSwervePosition(). They
        were in degrees before to enable the use of common trigonomoetry. We
        get Nics from degrees by calling getSwerveRotatingPosition().
        */
        m_frontRight->AssumeSwervePosition(m_frontRight->AbsoluteVectorToNics(frontRightResultVector, angle));
        m_frontLeft->AssumeSwervePosition(m_frontLeft->AbsoluteVectorToNics(frontLeftResultVector, angle));
        m_rearLeft->AssumeSwervePosition(m_rearLeft->AbsoluteVectorToNics(rearLeftResultVector, angle));
        m_rearRight->AssumeSwervePosition(m_rearRight->AbsoluteVectorToNics(rearRightResultVector, angle));

        m_frontRight->SetDriveSpeed(frontRightResultVector.magnitude() * throttle);
        m_frontLeft->SetDriveSpeed(frontLeftResultVector.magnitude() * throttle);
        m_rearLeft->SetDriveSpeed(rearLeftResultVector.magnitude() * throttle);
        m_rearRight->SetDriveSpeed(rearRightResultVector.magnitude() * throttle);
    }
}

void SwerveTrain::Log(std::string msg) {

    static std::vector<std::string> headers = {"SwerveTrain"};
    Logger::Log(msg, headers);
}