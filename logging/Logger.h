#pragma once

#include <string>
#include <iostream>
#include <vector>

class Logger {

    public:
        static void Log(std::string msg, std::vector<std::string> headers) {

            std::string line = "";
            for (std::string header : headers) {

                line += "[" + header + "]";
            }
            if (headers.size() > 0) {

                line += " ";
            }
            line += msg;
            std::cout << line << std::endl;
        }
};