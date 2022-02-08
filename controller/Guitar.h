#pragma once

#include <frc/Joystick.h>
#include <frc/Errors.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "controller/ControllerConsts.h"

class Guitar {

    public:
        Guitar(const int port) {

            m_controller = new frc::Joystick(port);
            m_s_t = 0;
        }

        void Update() {

            static frc::Timer timer;
            static bool prev = false;
            static bool toggle = false;
            static double prevSt = 0;
            static int counter = 0;

            bool current = !((m_controller->GetPOV(0) == -1) || (m_controller->GetPOV(0) == 270));
            if (current && !prev) {
                
                double delta = timer.Get().value();
                timer.Reset();
                timer.Start();
                double rate = delta != 0 ? 1.0 / delta : 0;
                // s_t = s_t-1 + a * (x_t - s_t-1)
                m_s_t = prevSt + R_smoothingFactor * (rate - prevSt);
            }
            else if (timer.Get().value() >= 1.0) {

                timer.Reset();
                timer.Start();
                m_s_t = 0;
            }
            prevSt = m_s_t;
            prev = current;
        }

        double StrumVelocity() {

            return m_s_t > 15.0 ? 1 : m_s_t / 15.0;
        }

    private:
        frc::Joystick* m_controller;
        double m_s_t;
};