#ifndef INVKIN_HPP_
#define INVKIN_HPP_

#include <eeros/math/Matrix.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/Block.hpp>
#include <eeros/control/Gain.hpp>

using namespace eeros::control;

class InvKin : public Block
{
public:
    InvKin(double B, double L)
        : RRG({1.0, 0.0, 0.0, 1.0}),
          RJT({1.0, 0.0, 0.0, 1.0 / L}),
          WJR({1.0, 1.0, -B / 2.0, B / 2.0})
    {
        // Name all blocks
        RRG.setName("invKin->RRG");
        RJT.setName("InvKin->RJT");
        WJR.setName("InvKin->WJR");

        // Name all signals
        RRG.getOut().getSignal().setName("TCP velocity setpoint [m/s] in the robot frame");
        RJT.getOut().getSignal().setName("{Robot velocity setpoint [m/s], "
                                         "robot angular velocity setpoint [rad/s]} in the robot frame");
        WJR.getOut().getSignal().setName("Wheel velocities [m/s]");

        // Connect all signals
        RJT.getIn().connect(RRG.getOut());
        WJR.getIn().connect(RJT.getOut());
    }

    // Input getter function
    Input<eeros::math::Vector2> &getInGvT() { return RRG.getIn(); }
    Input<> &getInPhi() { return phi; }

    // Output getter function
    Output<eeros::math::Vector2> &getOut() { return WJR.getOut(); }

    virtual void run()
    {
        RRG.setGain({cos(phi.getSignal().getValue()), -sin(phi.getSignal().getValue()),
                     sin(phi.getSignal().getValue()), cos(phi.getSignal().getValue())});
        RRG.run();
        RJT.run();
        WJR.run();
    }

    // Used Blocks
    Gain<eeros::math::Vector2, eeros::math::Matrix<2, 2>> RRG, RJT, WJR;
    Input<> phi;
};

#endif //INVKIN_HPP_
