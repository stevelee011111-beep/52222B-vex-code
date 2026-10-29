#include "lemlib/util.hpp"
#include <cmath>

namespace lemlib {
ExpoDriveCurve::ExpoDriveCurve(float deadband, float minOutput, float curve)
    : deadband(deadband),
      minOutput(minOutput),
      curveGain(curve) {}

float ExpoDriveCurve::curve(float input, float maxVelo) {
    // return 0 if input is within deadzone
    if (std::fabs(input) <= deadband) 
        return 0;
    // g is the output of g(x) as defined in the Desmos graph
    const float g = std::fabs(input) - deadband;
    // gmaxVelo is the output of g(maxVelo) as defined in the Desmos graph
    const float gmaxVelo = maxVelo - deadband;
    // i is the output of i(x) as defined in the Desmos graph
    const float i = std::pow(curveGain, g - maxVelo) * g * sgn(input);
    // imaxVelo is the output of i(maxVelo) as defined in the Desmos graph
    const float imaxVelo = std::pow(curveGain, gmaxVelo - maxVelo) * gmaxVelo;
    return (maxVelo - minOutput) / (maxVelo) * i * maxVelo / imaxVelo + minOutput * sgn(input);
}
} // namespace lemlib