#pragma once

#include <math.h>

class GeoUtils {

    public:
        static double MinDistFromDelta(const double delta, const double max) {

            if (abs(delta) > max / 2.0) {

                if (delta > 0) {

                    return -(max - delta);
                }
                else {

                    return max - (-delta);
                }
            }
            else {

                return delta;
            }
        }
};