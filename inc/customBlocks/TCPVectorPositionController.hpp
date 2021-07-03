#ifndef TCPVECTORPOSITIONCONTROLLER_HPP_
#define TCPVECTORPOSITIONCONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/Gain.hpp>
#include "customBlocks/EuclideanNorm.hpp"
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros::control;

class TCPVectorPositionController : public Block
{
public:
    TCPVectorPositionController(double f, double D, double tol, double maxVel)
        : GrTs({0.0, 0.0}), Kp(M_PI * f / D), vMax(maxVel), tol(tol), GvTc(this), enabled(false)
    {
        // Name all blocks
        GrTs.setName("TCPVectorPosCon->GrTs");
        GeTs.setName("TCPVectorPosCon->GeTs");
        GeTsNorm.setName("TCPVectorPosCon->GeTsNorm");
        Kp.setName("TCPVectorPosCon->Kp");
        vMax.setName("TCPVectorPosCon->vMax");

        // Name all signals
        GrTs.getOut().getSignal().setName("TCP position setpoint [m]");
        GeTs.getOut().getSignal().setName("TCP position error [m]");
        GeTsNorm.getOut().getSignal().setName("Length of TCP position error vector [m]");
        Kp.getOut().getSignal().setName("Controller output velocity [m/s]");
        vMax.getOut().getSignal().setName("Saturated controller output velocity [m/s]");
        GvTc.getSignal().setName("Controller output velocity as a vector [m/s]");

        // Connect signals
        GeTs.getIn(0).connect(GrTs.getOut());
        GeTs.negateInput(1);
        GeTsNorm.getIn().connect(GeTs.getOut());
        Kp.getIn().connect(GeTsNorm.getOut());
        vMax.getIn().connect(Kp.getOut());
    }

    Input<eeros::math::Vector2> &getIn() { return GeTs.getIn(1); }
    Output<eeros::math::Vector2> &getOut() { return GvTc; }

    virtual void run()
    {
        GrTs.run();
        GeTs.run();
        GeTsNorm.run();
        Kp.run();
        vMax.run();
        eeros::math::Vector2 uv;
        if (!getStatus() && enabled)
            GvTc.getSignal().setValue(vMax.getOut().getSignal().getValue() *
                                      GeTs.getOut().getSignal().getValue() /
                                      GeTsNorm.getOut().getSignal().getValue());
        else
            GvTc.getSignal().setValue(0.0);
        GvTc.getSignal().setTimestamp(eeros::System::getTimeNs());
    }

    void setTarget(eeros::math::Vector2 target) { GrTs.setValue(target); }

    bool getStatus() { return GeTsNorm.getOut().getSignal().getValue() < tol; }

    void enable() { enabled = true; }
    void disable() { enabled = false; }

    Constant<eeros::math::Vector2> GrTs;
    Sum<2, eeros::math::Vector2> GeTs;
    EuclideanNorm<> GeTsNorm;
    Gain<> Kp;
    Saturation<> vMax;
    Output<eeros::math::Vector2> GvTc;
    double tol;
    bool enabled;
};

#endif //TCPVECTORPOSITIONCONTROLLER_HPP_
