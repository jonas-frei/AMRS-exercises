#ifndef FWKINODOM_HPP_
#define FWKINODOM_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Block1i.hpp>
#include <eeros/control/I.hpp>
#include <eeros/control/Output.hpp>

using namespace eeros::control;

class FwKinOdom : public Block1i<eeros::math::Vector2>
{
public:
    FwKinOdom(double B, eeros::math::Vector2 GrRStart = 0.0, double phiStart = 0.0)
        : B(B)
    {
        // Name all blocks
        GrR.setName("FwKinOdom->GrR");
        phi.setName("FwKinOdom->phi");

        // Name all signals
        RvRx.getSignal().setName("Robot velocity in x direction in the robot frame [m/s]");
        omegaR.getSignal().setName("Robot angular velocity around the z Axis [rad/s]");
        GvR.getSignal().setName("Robot velocity in the global frame [m/s]");
        GrR.getOut().getSignal().setName("Robot position in the global frame [m]");
        phi.getOut().getSignal().setName("Robot orientation in the global frame [rad]");

        // Connect all signals
        GrR.getIn().connect(GvR);
        phi.getIn().connect(omegaR);

        // Additional block configuration
        GrR.setInitCondition(GrRStart);
        phi.setInitCondition(phiStart);
    }

    // Output getter functions
    Output<eeros::math::Vector2> &getOutGvR() { return GvR; }
    Output<eeros::math::Vector2> &getOutGrR() { return GrR.getOut(); }
    Output<> &getOutphi() { return phi.getOut(); }
    Output<> &getOutRvRx() { return RvRx; }
    Output<> &getOutomegaR() { return omegaR; }

    virtual void run()
    {
        // run the blocks in their respective order
        RvRx.getSignal().setValue((this->getIn().getSignal().getValue()[0] + this->getIn().getSignal().getValue()[1]) / 2.0);
        RvRx.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        omegaR.getSignal().setValue((this->getIn().getSignal().getValue()[1] - this->getIn().getSignal().getValue()[0]) / B);
        omegaR.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        phi.run();
        GvR.getSignal().setValue({cos(phi.getOut().getSignal().getValue()) * RvRx.getSignal().getValue(),
                                  sin(phi.getOut().getSignal().getValue()) * RvRx.getSignal().getValue()});
        GvR.getSignal().setTimestamp(RvRx.getSignal().getTimestamp());
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

    // Set position and orientation
    void setPose(eeros::math::Vector2 GrRStart, double phiStart)
    {
        GrR.setInitCondition(GrRStart);
        phi.setInitCondition(phiStart);
    }

protected:
    double B;
    Output<> RvRx, omegaR;
    Output<eeros::math::Vector2> GvR;
    I<eeros::math::Vector2> GrR;
    I<> phi;
};

#endif //FWKINODOM_HPP_
