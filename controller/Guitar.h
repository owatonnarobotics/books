#pragma once

#include <frc/XboxController.h>
#include <frc/Errors.h>
#include <frc/Timer.h>

#include "controller/ControllerConsts.h"

class Guitar {

    public:
        Guitar(const int port) {

            m_controller = new frc::XboxController(port);
            m_rate = 0;
        }

        void Update() {

            static frc::Timer timer;
            static bool prev = false;

            bool current = m_controller->GetRawButton(0);
            if (current && !prev) {
                
                units::time::second_t delta = timer.Get();
                timer.Reset();
                m_rate = 1.0 / delta.value();
            }
            prev = current;
        }

        double StrumVelocity() {

            static double s_t = 0;

            // s_t = s_t-1 + a * (x_t - s_t-1)
            s_t = s_t + R_smoothingFactor * (m_rate - s_t);
            return s_t;
        }

    private:
        frc::XboxController* m_controller;
        double m_rate;
};