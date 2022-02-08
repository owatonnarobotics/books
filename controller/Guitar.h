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
            m_rate = 0;
        }

        void Update() {

            static frc::Timer timer;
            static bool initialStart = false;
            static bool prev = false;
            static int counter = 0;

            if (!initialStart) {

                timer.Start();
                initialStart = true;
            }

            if (timer.Get().value() < R_samplingWindow) {
                
                bool current = !((m_controller->GetPOV(0) == -1) || (m_controller->GetPOV(0) == 270));
                if (current && !prev) {
                    
                    counter++;
                }
                prev = current;
            }
            else {

                timer.Reset();
                timer.Start();
                m_rate = counter;
                counter = 0;
            }
            frc::SmartDashboard::PutNumber("counter", counter);
        }

        double StrumVelocity() {

            frc::SmartDashboard::PutNumber("m_rate", m_rate);
            return m_rate > R_maxStrumsPerSecond * R_samplingWindow ? 1.0 : m_rate / ((double)R_maxStrumsPerSecond * R_samplingWindow);
        }

    private:
        frc::Joystick* m_controller;
        double m_rate;
};