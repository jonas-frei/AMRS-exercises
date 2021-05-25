#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Signal.hpp>
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
        // Name all blocks
        phi.setName("FwKinOdom->phi");
        GrR.setName("FwKinOdom->GrR");

        // Name all signals
        RvRx.setName("Robot velocity in x direction in the robot frame [m/s]");
        omegaR.getSignal().setName("Robot angular velocity around the z Axis [rad/s]");
        GvR.getSignal().setName("Robot velocity in the global frame [m/s]");
        GrR.getOut().getSignal().setName("Robot position in the global frame [m]");
        phi.getOut().getSignal().setName("Robot orientation in the global frame [rad]");

        // Connect all signals
        phi.getIn().connect(omegaR);
        phi.setInitCondition(0.0);
        GrR.getIn().connect(GvR);
        GrR.setInitCondition(0.0);
    }

    // Output getter functions
    Output<eeros::math::Vector2> &getOutGvR() { return GvR; }
    Output<eeros::math::Vector2> &getOutGrR() { return GrR.getOut(); }
    Output<> &getOutphi() { return phi.getOut(); }
    Output<> &getOutomegaR() { return omegaR; }

    virtual void run()
    {
        // run the blocks in their respective order
        RvRx.setValue((this->getIn().getSignal().getValue()[0] + this->getIn().getSignal().getValue()[1]) / 2.0);
        RvRx.setTimestamp(this->getIn().getSignal().getTimestamp());
        omegaR.getSignal().setValue((this->getIn().getSignal().getValue()[1] - this->getIn().getSignal().getValue()[0]) / B);
        omegaR.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        phi.run();
        GvR.getSignal().setValue({cos(phi.getOut().getSignal().getValue()) * RvRx.getValue(),
                                  sin(phi.getOut().getSignal().getValue()) * RvRx.getValue()});
        GvR.getSignal().setTimestamp(RvRx.getTimestamp());
        GrR.run();
    }

    // Enable the integrators
    void enableIntegrators(void)
    {
        GrR.enable();
        phi.enable();
    }

    // Disable the integrators
    void disableIntegrators(void)
    {
        GrR.disable();
        phi.disable();
    }

protected:
    double B;
    Signal<> RvRx;
    Output<> omegaR;
    Output<eeros::math::Vector2> GvR;
    I<eeros::math::Vector2> GrR;
    I<> phi;
};

#endif //FWKINODOM_HPP_
