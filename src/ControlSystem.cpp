#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"), E2("enc2"),
      controller(1.0 / dt, 0.7, 4.4, 6.8e-8 * 3441.0 / 104.0 * 3441.0 / 104.0),
      QMax(0.1),
      i_inv(104.0 / 3441.0),
      kM_inv(1 / 8.44e-3),
      R(8.0),
      qdMax(21.3),
      i(3441.0 / 104.0),
      kM(8.44e-3),
      M1("motor1"), M2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    E.setName("E");
    E_set.setName("E_set");
    controller.setName("controller");
    QMax.setName("QMax");
    i_inv.setName("i_inv");
    kM_inv.setName("kM_inv");
    R.setName("R");
    qdMax.setName("qdMax");
    i.setName("i");
    kM.setName("kM");
    U.setName("U");
    M.setName("M");
    M1.setName("M1");
    M2.setName("M2");

    // Name all signals
    E1.getOut().getSignal().setName("Encoder 1 position [rad]");
    E2.getOut().getSignal().setName("Encoder 2 position [rad]");
    E.getOut().getSignal().setName("Encoder positions [rad]");
    E_set.getOut().getSignal().setName("Switched encoder positions for controller setpoints [rad]");
    controller.getOut(0).getSignal().setName("Output shaft torque setpoints [Nm]");
    controller.getOut(1).getSignal().setName("Output shaft velocity setpoints [rad/s]");
    QMax.getOut().getSignal().setName("Saturated output shaft torque setpoints [Nm]");
    i_inv.getOut().getSignal().setName("Motor torque setpoints [Nm]");
    kM_inv.getOut().getSignal().setName("Motor current setpoints [A]");
    R.getOut().getSignal().setName("Motor voltage setpoints from pd controller [V]");
    qdMax.getOut().getSignal().setName("Saturated output shaft velocity setpoints [rad/s]");
    i.getOut().getSignal().setName("Motor velocity setpoints [rad/s]");
    kM.getOut().getSignal().setName("Motor voltage setpoints from feed forward [V]");
    U.getOut().getSignal().setName("Motor voltage setpoints [V]");
    M.getOut(0).getSignal().setName("Motor 1 voltage setpoint [V]");
    M.getOut(1).getSignal().setName("Motor 2 voltage setpoint [V]");

    // Connect signals
    E.getIn(0).connect(E1.getOut());
    E.getIn(1).connect(E2.getOut());
    E_set.getIn(0).connect(E2.getOut());
    E_set.getIn(1).connect(E1.getOut());
    controller.getIn(0).connect(E_set.getOut());
    controller.getIn(1).connect(E.getOut());
    QMax.getIn().connect(controller.getOut(0));
    i_inv.getIn().connect(QMax.getOut());
    kM_inv.getIn().connect(i_inv.getOut());
    R.getIn().connect(kM_inv.getOut());
    qdMax.getIn().connect(controller.getOut(1));
    i.getIn().connect(qdMax.getOut());
    kM.getIn().connect(i.getOut());
    U.getIn(0).connect(R.getOut());
    U.getIn(1).connect(kM.getOut());
    M.getIn().connect(U.getOut());
    M1.getIn().connect(M.getOut(0));
    M2.getIn().connect(M.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(E);
    timedomain.addBlock(E_set);
    timedomain.addBlock(controller);
    timedomain.addBlock(QMax);
    timedomain.addBlock(i_inv);
    timedomain.addBlock(kM_inv);
    timedomain.addBlock(R);
    timedomain.addBlock(qdMax);
    timedomain.addBlock(i);
    timedomain.addBlock(kM);
    timedomain.addBlock(U);
    timedomain.addBlock(M);
    timedomain.addBlock(M1);
    timedomain.addBlock(M2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}