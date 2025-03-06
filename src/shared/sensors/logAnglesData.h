#include <iostream>

struct LogAngles
{
    float roll, pitch, yaw;

    LogAngles() : roll{0}, pitch{0}, yaw(0) {}
    LogAngles(float Roll, float Pitch, float Yaw)
        : roll{Roll}, pitch{Pitch}, yaw(Yaw)
    {
    }

    void getAngles(float x, float y, float z)
    {
        this->roll  = x * (180 / M_PI);
        this->pitch = y * (180 / M_PI);
        this->yaw   = z * (180 / M_PI);
    }

    static std::string header() { return "Roll,Pitch,Yaw,quaternionZ\n"; }

    void print(std::ostream& os) const
    {
        os << roll << "," << pitch << "," << yaw << "\n";
    }
};
