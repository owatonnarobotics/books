/*
class NavX

Constructors

    NavX(const int&)
        Creates a NavX on the specified interface (kUSB, kMXP)

Public Methods

    double getYaw()
        Returns the yaw value.
    double getYawFull()
        Returns the yaw value from 0-360.
        Rolls over at extremes; used with the standard unit circle.
    double getAngle()
        Returns the angle value (-infinity to infinity, beginning at 0).
    double getAbsoluteAngle()
        Returns the absolute value of the angle value.
    void resetYaw()
        Sets the yaw value to zero.
    void resetAll()
        Resets all NavX return values and calibrates the sensor.

    enum ConnectionType
        Used with the constructor to specify which interface to construct on.
*/

#pragma once

#include <math.h>

#include "AHRS.h"

class NavX {

    public:
        NavX(const int &connectionType) {

            if (connectionType == kUSB) {

                navX = new AHRS(frc::SPI::kOnboardCS0);
            }
            else if (connectionType == kMXP) {

                navX = new AHRS(frc::SPI::kMXP);
            }
            else {

                navX = new AHRS(frc::SPI::kMXP);
            }
        }

        double getYaw() {

            return navX->GetYaw();
        }
        double getYawFull(){

            if (getYaw() < 0) {

                return getYaw() + 360;
            }
            else {

                return getYaw();
            }
        }
        double getAngle() {

            return navX->GetAngle();
        }
        double getAbsoluteAngle() {

            return abs(navX->GetAngle());
        }

        void resetYaw() {

            navX->ZeroYaw();
        }
        void resetAll() {

            navX->Reset();
        }

        enum ConnectionType {

            kUSB,
            kMXP = 4
        };

    private:
        AHRS *navX;
};
