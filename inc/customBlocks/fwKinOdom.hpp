#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Output.hpp>

using namespace eeros::control;

class FwKinOdom : public Block1i<eeros::math::Vector2>
{
public:
    FwKinOdom(double B)
        : B(B)
    {
        phi.getIn().connect(omegaR);
        phi.setInitCondition(0.0);
        GrR.getIn().connect(GvR);
        GrR.setInitCondition(0.0);
    }

    Output<eeros::math::Vector2> &getOutGvR() { return GvR; }
    Output<eeros::math::Vector2> &getOutGrR() { return GrR.getOut(); }
    Output<> &getOutphi() { return phi.getOut(); }
    Output<> &getOutomegaR() { return omegaR; }
    

    virtual void run()
    {
        RvRx = (this->getIn().getSignal().getValue()[0] + this->getIn().getSignal().getValue()[1]) / 2.0;
        omegaR.getSignal().setValue((this->getIn().getSignal().getValue()[1] - this->getIn().getSignal().getValue()[0]) / B);
        omegaR.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        phi.run();
        GvR.getSignal().setValue({cos(phi.getOut().getSignal().getValue()) * RvRx,
                                  sin(phi.getOut().getSignal().getValue()) * RvRx});
        GvR.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        GrR.run();
    }

    void enableIntegrators(void)
    {
        GrR.enable();
        phi.enable();
    }

    void disableIntegrators(void)
    {
        GrR.disable();
        phi.disable();
    }

protected:
    double RvRx, B;
    Output<> omegaR;
    Output<eeros::math::Vector2> GvR;
    I<eeros::math::Vector2> GrR;
    I<> phi;
};

#endif //FWKINODOM_HPP_
