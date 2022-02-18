#ifndef AUTOSEQUENCE_H
#define AUTOSEQUENCE_H

#include <vector>

#include "commonauto/AutoStep.h"
#include "logging/Logger.h"

class AutoSequence : public AutoStep {

    public:
        AutoSequence(const bool &loop) : AutoStep("AutoSequence") {

            m_loop = loop;
            m_log = false;
        }

        void Init() {

            if (!m_steps.empty()) {
                m_currentStep = m_steps.begin();
                m_lastStep = m_steps.back();
                (*m_currentStep)->Init();
                m_timer = Now();
                Log("Initialized: " + (*m_currentStep)->GetName());
                m_done = false;
            }
            else {

                m_done = true;
            }
        }

        bool Execute() {

            if (!m_done) {
                
                // If the current step has finished
                if ((*m_currentStep)->Execute()) {

                    Log((*m_currentStep)->GetName() + " has finished; time of execution: " + std::to_string(Now() - m_timer) + " s");

                    // If the step that just finished is the last step
                    if ((*m_currentStep) == m_lastStep) {

                        Log("End of sequence");
                        
                        // If we should loop
                        if (m_loop) {
                            
                            Log("Looping");

                            // Set the current step to the first step
                            m_currentStep = m_steps.begin();
                            // Initialize the first step
                            (*m_currentStep)->Init();
                            Log("Initialized: " + (*m_currentStep)->GetName());
                        }
                        else {

                            // If we shouldn't loop, this AutoSequence is done
                            m_done = true;
                        }
                    }
                    else {

                        // Move on to the next step
                        m_currentStep++;
                        // Initialize the next step
                        (*m_currentStep)->Init();
                        Log("Initialized: " + (*m_currentStep)->GetName());
                        m_timer = Now();
                    }
                }
            }
            return m_done;
        }

        void AddStep(AutoStep* refStep) {

            m_steps.push_back(refStep);
        }

        void Reset() {

            m_steps.clear();
        }

        void EnableLogging() {

            m_log = true;
        }

        void DisableLogging() {

            m_log = false;
        }

        double Now() {

            return frc::GetTime().value();
        }

    private:
        std::vector<AutoStep*> m_steps;
        std::vector<AutoStep*>::iterator m_currentStep;
        AutoStep* m_lastStep;
        bool m_done;
        bool m_loop;
        bool m_log;
        double m_timer;

        void Log(std::string msg) {

            if (m_log) {
                
                std::vector<std::string> headers = {"Auto", "AutoSequence"};
                Logger::Log(msg, headers);
            }
        }
};

#endif