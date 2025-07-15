// TODO Copyright

#pragma once

#include <sensors/correction/BiasCorrector/BiasCorrector.h>

#include <Eigen/Core>
#include <string>

namespace Boardcore
{

class TwelveParametersCorrector : public BiasCorrector
{
    TwelveParametersCorrector(Eigen::Matrix3f W, Eigen::Vector3f V);
    TwelveParametersCorrector();

    bool fromFile(const std::string& filename) override;
    bool toFile(const std::string& filename) override;

    Eigen::Vector3f correct(const Eigen::Vector3f& inputVector) const override;

    Eigen::Vector3f getV() const;
    void setV(const Eigen::Vector3f& V);

    Eigen::Matrix3f getW() const;
    void setW(const Eigen::Matrix3f& W);

protected:
    Eigen::Vector3f V;
    Eigen::Matrix3f W;
};

}  // namespace Boardcore
