#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      myConstant(1.0),
      Mwl("motor1"), Mwr("motor2"),

      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    Ewl.setName("Ewl");
    Ewr.setName("Ewr");
    myConstant.setName("My constant");
    Mwl.setName("Mwl");
    Mwr.setName("Mwr");

    // Name all signals
    Ewl.getOut().getSignal().setName("Position left wheel [m]");
    Ewr.getOut().getSignal().setName("Position right wheel [m]");
    myConstant.getOut().getSignal().setName("Voltage motor right wheel [V]");

    // Connect signals
    Mwl.getIn().connect(myConstant.getOut());
    Mwr.getIn().connect(myConstant.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(myConstant);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}