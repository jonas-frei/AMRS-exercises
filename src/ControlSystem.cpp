#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      myConstant({0.2, 0.4}),
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
    mux.setName("mux");
    myConstant.setName("My constant");
    piController.setName("PI Controller");
    invMotMod.setName("invMotMod");
    deMux.setName("deMux");
    Mwl.setName("Mwl");
    Mwr.setName("Mwr");

    // Name all signals
    Ewl.getOut().getSignal().setName("Left wheel position [m]");
    Ewr.getOut().getSignal().setName("Right wheel position [m]");
    mux.getOut().getSignal().setName("Wheel positions [m]");
    myConstant.getOut().getSignal().setName("Velocity setpoints [V]");
    deMux.getOut(0).getSignal().setName("Left motor voltage setpoint [V]");
    deMux.getOut(0).getSignal().setName("Right motor voltage setpoint [V]");

    // Connect signals
    Dwl.getIn().connect(Ewl.getOut());
    Dwr.getIn().connect(Ewr.getOut());
    mux.getIn(0).connect(Dwl.getOut());
    mux.getIn(1).connect(Dwr.getOut());
    piController.getInqds().connect(myConstant.getOut());
    piController.getInqd().connect(mux.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    deMux.getIn().connect(invMotMod.getOutU());
    Mwl.getIn().connect(deMux.getOut(0));
    Mwr.getIn().connect(deMux.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(Dwl);
    timedomain.addBlock(Dwr);
    timedomain.addBlock(mux);
    timedomain.addBlock(myConstant);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(deMux);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}