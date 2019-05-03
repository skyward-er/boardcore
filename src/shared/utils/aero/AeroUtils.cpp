#include "AeroUtils.h"

namespace aeroutils
{

float relAltitude(float pressure, float pressure_ref, float temperature_ref)
{
    using namespace constants;

    return temperature_ref / a * (1 - powf(pressure / pressure_ref, n_inv));
}

float mslPressure(float pressure_ref, float temperature_ref, float altitude_ref)
{
    using namespace constants;
    float T0 = mslTemperature(temperature_ref, altitude_ref);

    return pressure_ref / powf(1 - a * altitude_ref / T0, n);
}

float mslTemperature(float temperature_ref, float altitude_ref)
{
    return temperature_ref + (altitude_ref * constants::a);
}

float verticalSpeed(float p, float dp_dt, float p_ref, float t_ref)
{
    using namespace constants;

    return -(t_ref * dp_dt * powf(p / p_ref, n_inv)) / (a * n * p);
}
}  // namespace aeroutils