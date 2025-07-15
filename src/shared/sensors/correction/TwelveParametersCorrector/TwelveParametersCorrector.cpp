#include "TwelveParametersCorrector.h"

#include <fstream>

namespace Boardcore
{

TwelveParametersCorrector::TwelveParametersCorrector(Eigen::Matrix3f W,
                                                     Eigen::Vector3f V)
    : BiasCorrector(V), W(W)
{
}

TwelveParametersCorrector::TwelveParametersCorrector() { W.setIdentity(); }

bool TwelveParametersCorrector::fromFile(const std::string& filename)
{
    std::ifstream inputFile(filename);

    if (inputFile)
    {
        inputFile.ignore(1000, '\n');

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                inputFile >> W(i, j);
                inputFile.ignore(1, ',');
            }
        }

        for (int i = 0; i < 3; i++)
        {
            inputFile >> V(i);
            inputFile.ignore(1, ',');
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool TwelveParametersCorrector::toFile(const std::string& filename)
{
    std::ofstream outputFile(filename);

    if (outputFile)
    {
        outputFile << "w00,w01,w02,w10,w11,w12,w20,w21,w22,v0,v1,v2"
                   << std::endl;
        outputFile << W(0, 0) << "," << W(0, 1) << "," << W(0, 2) << ","
                   << W(1, 0) << "," << W(1, 1) << "," << W(1, 2) << ","
                   << W(2, 0) << "," << W(2, 1) << "," << W(2, 2) << "," << V(0)
                   << "," << V(1) << "," << V(2) << std::endl;
        outputFile.close();
        return 1;
    }
    else
    {
        return 0;
    }
}

Eigen::Vector3f TwelveParametersCorrector::correct(
    const Eigen::Vector3f& inputVector) const
{
    return (W * inputVector) - V;
}

Eigen::Vector3f TwelveParametersCorrector::getV() const { return V; }

void TwelveParametersCorrector::setV(const Eigen::Vector3f& V) { this->V = V; }

Eigen::Matrix3f TwelveParametersCorrector::getW() const { return W; }

void TwelveParametersCorrector::setW(const Eigen::Matrix3f& W) { this->W = W; }

}  // namespace Boardcore
