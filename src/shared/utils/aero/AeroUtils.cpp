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
    return temperature_ref - (altitude_ref * constants::a);
}
}  // namespace aeroutils