#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double ts)
    : myConstant(1.0), myGain(2.0),
      timedomain("Main time domain", ts, true)
{
    // Name all signals
    myConstant.getOut().getSignal().setName("My constant value");
    myGain.getOut().getSignal().setName("My constant value multiplied with my gain");

    // Connect signals
    myGain.getIn().connect(myConstant.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(myConstant);
    timedomain.addBlock(myGain);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}