#ifndef INVMOTMOD_HPP_
#define INVMOTMOD_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Sum.hpp>

using namespace eeros::control;

template <typename T = double>
class InvMotMod : public Block
{
public:
    InvMotMod(double QMax, double qdMax, double i, double kM, double R)
        : QMax(QMax),
          qdMax(qdMax),
          iInv(1.0 / i),
          kMInv(1.0 / kM),
          R(R),
          i(i),
          kM(kM)
    {
        // Name all blocks
        this->QMax.setName("InvMotMod->QMax");
        this->qdMax.setName("InvMotMod->qdMax");
        this->iInv.setName("InvMotMod->iInv");
        this->kMInv.setName("InvMotMod->kMInv");
        this->R.setName("InvMotMod->R");
        this->i.setName("InvMotMod->i");
        this->kM.setName("InvMotMod->kM");
        this->U.setName("InvMotMod->U");

        // Name all signals
        this->QMax.getOut().getSignal().setName("Saturated wheel force setpoint [N]");
        this->qdMax.getOut().getSignal().setName("Saturated wheel velocity setpoint [m/s]");
        this->iInv.getOut().getSignal().setName("Motor torque setpoint [Nm]");
        this->kMInv.getOut().getSignal().setName("Motor current setpoint [A]");
        this->R.getOut().getSignal().setName("Motor voltage setpoint from force [V]");
        this->i.getOut().getSignal().setName("Motor angular velocity setpoint [rad/s]");
        this->kM.getOut().getSignal().setName("Motor voltage setpoint from velocity [V]");
        this->U.getOut().getSignal().setName("Motor voltage setpoint [V]");

        // Connect all signals
        this->iInv.getIn().connect(this->QMax.getOut());
        this->kMInv.getIn().connect(this->iInv.getOut());
        this->R.getIn().connect(this->kMInv.getOut());
        this->i.getIn().connect(this->qdMax.getOut());
        this->kM.getIn().connect(this->i.getOut());
        this->U.getIn(0).connect(this->R.getOut());
        this->U.getIn(1).connect(this->kM.getOut());
    }

    // Input getter functions
    Input<T> &getInQ() { return QMax.getIn(); }
    Input<T> &getInqd() { return qdMax.getIn(); }

    // Output getter functions
    Output<T> &getOutU() { return U.getOut(); }

    virtual void run()
    {
        // run the blocks in their respective order
        QMax.run();
        iInv.run();
        kMInv.run();
        R.run();
        qdMax.run();
        i.run();
        kM.run();
        U.run();
    }

protected:
    // Used blocks
    Saturation<T> QMax, qdMax;
    Gain<T> iInv, kMInv, R, i, kM;
    Sum<2, T> U;
};

#endif //INVMOTMOD_HPP_
