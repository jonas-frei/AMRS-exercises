#ifndef PICONTROLLER_HPP_
#define PICONTROLLER_HPP_

#include <eeros/control/Block.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/I.hpp>

using namespace eeros::control;

template <typename T = double>
class PIController : public Block
{
public:
    // Contructor for symmetic limits
    PIController(double f_Task, double D, double s, double M, double ILimit)
        : Kp(f_Task / s),
          KI(f_Task / 2.0 / s / D * f_Task / 2.0 / s / D),
          M(M)
    {
        init(ILimit, -ILimit);
    }
    // Contructor for asymmetic limits
    PIController(double f_Task, double D, double s, double M, double IUpperLimit, double ILowerLimit)
        : Kp(f_Task / s),
          KI(f_Task / 2.0 / s / D * f_Task / 2.0 / s / D),
          M(M)
    {
        init(IUpperLimit, ILowerLimit);
    }

    // Input getter functions
    Input<T> &getInqds() { return ed.getIn(0); }
    Input<T> &getInqd() { return qd.getIn(); }

    // Output getter functions
    Output<T> &getOutQ() { return M.getOut(); }
    Output<T> &getOutqd() { return qd.getOut(); }

    virtual void run()
    {
        // run the blocks in their respective order
        qd.run();
        ed.run();
        Kp.run();
        e.run();
        KI.run();
        qddC.run();
        M.run();
    }

    // Enable/disable the integrator
    void enableIntegrator(void) { e.enable(); }
    void disableIntegrator(void) { e.disable(); }

    // Used blocks
    Sum<2, T> ed, qddC;
    Gain<T> qd, Kp, KI, M;
    I<T> e;

private:
    void init(double IUpperLimit, double ILowerLimit)
    {
        // Name all Blocks
        ed.setName("PIController->ed");
        qddC.setName("PIController->qddC");
        qd.setName("PIController->qd");
        Kp.setName("PIController->Kp");
        KI.setName("PIController->KI");
        e.setName("PIController->e");
        M.setName("PIController->M");

        // Name all Signals
        ed.getOut().getSignal().setName("Wheel velocity error [m/s]");
        qddC.getOut().getSignal().setName("Wheel acceleration setpoint [m/s²]");
        qd.getOut().getSignal().setName("Wheel velocity setpoint [m/s]");
        Kp.getOut().getSignal().setName("Wheel acceleration setpoint from proportional controller part [m/s]");
        KI.getOut().getSignal().setName("Wheel acceleration setpoint from integrative controller part [m/s]");
        e.getOut().getSignal().setName("Wheel position error [m]");
        M.getOut().getSignal().setName("Wheel force setpoint [N]");

        // Connect all Signals
        ed.negateInput(1);
        ed.getIn(1).connect(qd.getOut());
        Kp.getIn().connect(ed.getOut());
        e.getIn().connect(ed.getOut());
        e.setInitCondition(0.0);
        e.setLimit(IUpperLimit, ILowerLimit);
        KI.getIn().connect(e.getOut());
        qddC.getIn(0).connect(KI.getOut());
        qddC.getIn(1).connect(Kp.getOut());
        this->M.getIn().connect(qddC.getOut());
    }
};

#endif // PICONTROLLER_HPP_
