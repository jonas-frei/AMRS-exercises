#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      myConstant(0.2), myConstant2(0.0),
      invMotMod(AMRSC::MOT::QMAX, AMRSC::MOT::qdMAX, AMRSC::MOT::i, AMRSC::MOT::KM, AMRSC::MOT::R),
      piController(1.0 / dt, AMRSC::CONT::D, AMRSC::CONT::s, AMRSC::CONT::M, AMRSC::CONT::ILIMIT),
      Mwl("motor1"), Mwr("motor2"),

      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    Ewl.setName("Ewl");
    Ewr.setName("Ewr");
    Dwl.setName("Dwl");
    Dwr.setName("Dwr");
    myConstant.setName("My constant");
    piController.setName("PI Controller");
    invMotMod.setName("invMotMod");
    Mwl.setName("Mwl");
    Mwr.setName("Mwr");

    // Name all signals
    Ewl.getOut().getSignal().setName("Position left wheel [m]");
    Ewr.getOut().getSignal().setName("Position right wheel [m]");
    myConstant.getOut().getSignal().setName("Voltage motor right wheel [V]");

    // Connect signals
    Dwl.getIn().connect(Ewl.getOut());
    piController.getInqds().connect(myConstant.getOut());
    piController.getInqd().connect(Dwl.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    Mwl.getIn().connect(invMotMod.getOutU());
    Mwr.getIn().connect(invMotMod.getOutU());

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(Dwl);
    timedomain.addBlock(myConstant);
    timedomain.addBlock(myConstant2);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}