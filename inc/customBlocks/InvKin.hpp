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
    InvKin(double B)
        : WJR({1.0, 1.0, -B / 2.0, B / 2.0})
    {
        // Name all blocks
        mux.setName("InvKin->mux");
        WJR.setName("InvKin->WJR");

        // Name all signals
        mux.getOut().getSignal().setName("{Robot velocity setpoint [m/s], robot angular velocity setpoint [rad/s]} in the global frame");
        WJR.getOut().getSignal().setName("Wheel velocities [m/s]");

        // Connect all signals
        WJR.getIn().connect(mux.getOut());
    }

    // Input getter function
    Input<> &getIn(uint8_t index)
    {
        if (index >= 2)
            throw eeros::Fault("Index expeeds number of inputs in block '" + this->getName() + "'");
        return mux.getIn(index);
    }

    // Output getter function
    Output<eeros::math::Vector2> &getOut() { return WJR.getOut(); }

    virtual void run()
    {
        // Call the run method of the blocks in their respective order
        mux.run();
        WJR.run();
    }

    // Used Blocks
    Mux<2> mux;
    Gain<eeros::math::Vector2, eeros::math::Matrix<2, 2>> WJR;
};

#endif //INVKIN_HPP_
