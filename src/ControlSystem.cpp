#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      fwKinOdom(AMRSC::ROB::B),
      myConstant(0.0),
      myConstant2(1.0),
      invKin(AMRSC::ROB::B),
      piController(1.0 / dt, AMRSC::CONT::D, AMRSC::CONT::s, AMRSC::CONT::M, AMRSC::CONT::ILIMIT),
      invMotMod(AMRSC::MOT::QMAX, AMRSC::MOT::qdMAX, AMRSC::MOT::i, AMRSC::MOT::KM, AMRSC::MOT::R),
      Mwl("motor1"), Mwr("motor2"),

      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    Ewl.setName("Ewl");
    Ewr.setName("Ewr");
    vw.setName("vw");
    mux.setName("mux");
    fwKinOdom.setName("fwKinOdom");
    myConstant.setName("My constant");
    invKin.setName("invKin");
    piController.setName("PI Controller");
    invMotMod.setName("invMotMod");
    deMux.setName("deMux");
    Mwl.setName("Mwl");
    Mwr.setName("Mwr");

    // Name all signals
    Ewl.getOut().getSignal().setName("Left wheel position [m]");
    Ewr.getOut().getSignal().setName("Right wheel position [m]");
    mux.getOut().getSignal().setName("Wheel positions [m]");
    vw.getOut().getSignal().setName("Wheel velocities [m/s]");
    myConstant.getOut().getSignal().setName("Velocity setpoints [V]");
    deMux.getOut(0).getSignal().setName("Left motor voltage setpoint [V]");
    deMux.getOut(0).getSignal().setName("Right motor voltage setpoint [V]");

    // Connect signals
    mux.getIn(0).connect(Ewl.getOut());
    mux.getIn(1).connect(Ewr.getOut());
    vw.getIn().connect(mux.getOut());
    fwKinOdom.getIn().connect(vw.getOut());
    invKin.getIn(0).connect(myConstant.getOut());
    invKin.getIn(1).connect(myConstant2.getOut());
    piController.getInqds().connect(invKin.getOut());
    piController.getInqd().connect(vw.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    deMux.getIn().connect(invMotMod.getOutU());
    Mwl.getIn().connect(deMux.getOut(0));
    Mwr.getIn().connect(deMux.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(mux);
    timedomain.addBlock(vw);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(myConstant);
    timedomain.addBlock(myConstant2);
    timedomain.addBlock(invKin);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(deMux);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}