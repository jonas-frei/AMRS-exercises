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
    FwKinOdom(double B, double L, eeros::math::Vector2 GrRStart = 0.0, double phiStart = 0.0)
        : B(B), L(L), RvRx(this), omegaR(this), RvTx(this), RvTy(this), omegaT(this), GvT(this)
    {
        // Name all blocks
        GrT.setName("FwKinOdom->GrT");
        phi.setName("FwKinOdom->phi");

        // Name all signals
        RvRx.getSignal().setName("Robot velocity in x direction in the robot frame [m/s]");
        omegaR.getSignal().setName("Robot angular velocity around the z Axis [rad/s]");
        RvTx.getSignal().setName("Pivot velocity in x direction in the robot frame [m/s]");
        RvTy.getSignal().setName("Pivot velocity in y direction in the robot frame [m/s]");
        omegaT.getSignal().setName("Pivot angular velocity around the z Axis [rad/s]");
        GvT.getSignal().setName("Pivot velocity in the global frame [m/s]");
        GrT.getOut().getSignal().setName("Pivot position in the global frame [m]");
        phi.getOut().getSignal().setName("Pivot orientation in the global frame [rad]");

        // Connect all signals
        GrT.getIn().connect(GvT);
        phi.getIn().connect(omegaT);

        // Additional block configuration
        GrT.setInitCondition(GrRStart);
        phi.setInitCondition(phiStart);
    }

    // Output getter functions
    Output<eeros::math::Vector2> &getOutGvT() { return GvT; }
    Output<eeros::math::Vector2> &getOutGrT() { return GrT.getOut(); }
    Output<> &getOutphi() { return phi.getOut(); }
    Output<> &getOutRvRx() { return RvRx; }
    Output<> &getOutomegaR() { return omegaR; }
    Output<> &getOutRvTx() { return RvTx; }
    Output<> &getOutRvTy() { return RvTy; }
    Output<> &getOutOmegaT() { return omegaT; }

    virtual void run()
    {
        // run the blocks in their respective order
        RvRx.getSignal().setValue((this->getIn().getSignal().getValue()[0] + this->getIn().getSignal().getValue()[1]) / 2.0);
        RvRx.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        omegaR.getSignal().setValue((this->getIn().getSignal().getValue()[1] - this->getIn().getSignal().getValue()[0]) / B);
        omegaR.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        RvTx.getSignal().setValue(RvRx.getSignal().getValue());
        RvTx.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        RvTy.getSignal().setValue(omegaR.getSignal().getValue() * L);
        RvTy.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        omegaT.getSignal().setValue(omegaR.getSignal().getValue());
        omegaT.getSignal().setTimestamp(this->getIn().getSignal().getTimestamp());
        phi.run();
        GvT.getSignal().setValue({cos(phi.getOut().getSignal().getValue()) * RvTx.getSignal().getValue() -
                                      sin(phi.getOut().getSignal().getValue()) * RvTy.getSignal().getValue(),
                                  sin(phi.getOut().getSignal().getValue()) * RvTx.getSignal().getValue() +
                                      cos(phi.getOut().getSignal().getValue()) * RvTy.getSignal().getValue()});
        GvT.getSignal().setTimestamp(RvRx.getSignal().getTimestamp());
        GrT.run();
    }

    // Enable the integrators
    void enableIntegrators(void)
    {
        GrT.enable();
        phi.enable();
    }

    // Disable the integrators
    void disableIntegrators(void)
    {
        GrT.disable();
        phi.disable();
    }

    // Set position and orientation
    void setPose(eeros::math::Vector2 GrRStart, double phiStart)
    {
        GrT.setInitCondition(GrRStart);
        phi.setInitCondition(phiStart);
    }

protected:
    double B, L;
    Output<> RvRx, omegaR, RvTx, RvTy, omegaT;
    Output<eeros::math::Vector2> GvT;
    I<eeros::math::Vector2> GrT;
    I<> phi;
};

#endif //FWKINODOM_HPP_
