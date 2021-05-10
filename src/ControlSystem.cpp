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
      M1("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    controller.setName("controller");
    QMax.setName("QMax");
    i_inv.setName("i_inv");
    kM_inv.setName("kM_inv");
    R.setName("R");
    d.setName("d");
    qdMax.setName("qdMax");
    i.setName("i");
    kM.setName("kM");
    U.setName("U");
    M1.setName("M1");

    // Name all signals
    E1.getOut().getSignal().setName("Position encoder 1 [rad]");
    E2.getOut().getSignal().setName("Position encoder 2 [rad]");
    controller.getOut().getSignal().setName("Output shaft torque setpoint 1 [Nm]");
    QMax.getOut().getSignal().setName("Saturated output shaft torque setpoint 1 [Nm]");
    i_inv.getOut().getSignal().setName("Motor 1 torque setpoint [Nm]");
    kM_inv.getOut().getSignal().setName("Motor 1 setpoint current [A]");
    R.getOut().getSignal().setName("Motor 1 setpoint voltage from pd controller [V]");
    d.getOut().getSignal().setName("Output shaft velocity setpoint 1 [rad/s]");
    qdMax.getOut().getSignal().setName("Saturated output shaft velocity setpoint 1 [rad/s]");
    i.getOut().getSignal().setName("Motor 1 velocity setpoint [rad/s]");
    kM.getOut().getSignal().setName("Motor 1 setpoint voltage from feed forward [V]");
    U.getOut().getSignal().setName("Motor 1 setpoint voltage [V]");

    // Connect signals
    controller.getIn(0).connect(E2.getOut());
    controller.getIn(1).connect(E1.getOut());
    QMax.getIn().connect(controller.getOut());
    i_inv.getIn().connect(QMax.getOut());
    kM_inv.getIn().connect(i_inv.getOut());
    R.getIn().connect(kM_inv.getOut());
    d.getIn().connect(E1.getOut());
    qdMax.getIn().connect(d.getOut());
    i.getIn().connect(qdMax.getOut());
    kM.getIn().connect(i.getOut());
    U.getIn(0).connect(R.getOut());
    U.getIn(1).connect(kM.getOut());
    M1.getIn().connect(U.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(controller);
    timedomain.addBlock(QMax);
    timedomain.addBlock(i_inv);
    timedomain.addBlock(kM_inv);
    timedomain.addBlock(R);
    timedomain.addBlock(d);
    timedomain.addBlock(qdMax);
    timedomain.addBlock(i);
    timedomain.addBlock(kM);
    timedomain.addBlock(U);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}