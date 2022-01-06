#ifndef DRIVE_STATIC_H
#define DRIVE_STATIC_H

#include "commonauto/AutoStep.h"
#include "swerve/src/include/SwerveTrain.h"

class SwerveTrain;

class DriveStatic : public AutoStep {

    public:
        DriveStatic(
            const double &x,
            const double &y,
            const double rawZ,
            const bool &precision,
            const bool &relative,
            const bool &hold,
            const double throttle = 1.0
        ) : AutoStep("DriveStatic") {

            m_x = x;
            m_y = y;
            m_rawZ = rawZ;
            m_relative = relative;
            m_hold = hold;
            m_throttle = throttle;
        }

        void Init() {}

        bool Execute() {

            SwerveTrain::GetInstance().Drive(
                m_x,
                m_y,
                m_rawZ,
                m_relative,
                m_hold,
                m_throttle
            );

            return false;
        }

    private:
        double m_x;
        double m_y;
        double m_rawZ;
        bool m_precision;
        bool m_relative;
        bool m_hold;
        double m_throttle;
};

#endif