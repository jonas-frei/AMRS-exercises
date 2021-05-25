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
        this->QMax.setName("QMax");
        this->qdMax.setName("qdMax");
        this->iInv.setName("iInv");
        this->kMInv.setName("kMInv");
        this->R.setName("R");
        this->i.setName("i");
        this->kM.setName("kM");
        this->U.setName("U");

        // Name all signals
        this->QMax.getOut().getSignal().setName("Saturated wheel force setpoint [N]");
        this->qdMax.getOut().getSignal().setName("Saturated wheel velocity setpoint [m/s]");
        this->iInv.getOut().getSignal().setName("Motor torque setpoint [Nm]");
        this->kMInv.getOut().getSignal().setName("Motor current setpoint [A]");
        this->R.getOut().getSignal().setName("Motor voltage setpoint [V]");
        this->i.getOut().getSignal().setName("Motor angular velocity setpoint [rad/s]");
        this->kM.getOut().getSignal().setName("Motor voltage setpoint [V]");

        // Connect all signals
        this->iInv.getIn().connect(this->QMax.getOut());
        this->kMInv.getIn().connect(this->iInv.getOut());
        this->R.getIn().connect(this->kMInv.getOut());
        this->i.getIn().connect(this->qdMax.getOut());
        this->kM.getIn().connect(this->i.getOut());
        this->U.getIn(0).connect(this->R.getOut());
        this->U.getIn(1).connect(this->kM.getOut());
    }

    Input<T> &getInQ() { return QMax.getIn(); }
    Input<T> &getInqd() { return qdMax.getIn(); }

    Output<T> &getOutU() { return U.getOut(); }

    virtual void run()
    {
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
    Saturation<T> QMax, qdMax;
    Gain<T> iInv, kMInv, R, i, kM;
    Sum<2, T> U;
};

#endif //INVMOTMOD_HPP_
