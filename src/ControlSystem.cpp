#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      KFwl(AMRSC::KF::Ad, AMRSC::KF::Bd, AMRSC::KF::C, AMRSC::KF::Gd, AMRSC::KF::Q, AMRSC::KF::R),
      KFwr(AMRSC::KF::Ad, AMRSC::KF::Bd, AMRSC::KF::C, AMRSC::KF::Gd, AMRSC::KF::Q, AMRSC::KF::R),
      fwKinOdom(AMRSC::ROB::B, AMRSC::ROB::L),
      posController(AMRSC::CONT::fPos, AMRSC::CONT::D, AMRSC::CONT::posTol, AMRSC::CONT::vMax),
      invKin(AMRSC::ROB::B, AMRSC::ROB::L),
      piController(1.0 / dt, AMRSC::CONT::D, AMRSC::CONT::s, AMRSC::CONT::M, AMRSC::CONT::ILIMIT),
      invMotMod(AMRSC::MOT::QMAX, AMRSC::MOT::qdMAX, AMRSC::MOT::i, AMRSC::MOT::KM, AMRSC::MOT::R),
      Mwl("motor1"), Mwr("motor2"),

      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    Ewl.setName("Ewl");
    Ewr.setName("Ewr");
    KFwl.setName("KFwl");
    KFwr.setName("KFwr");
    vw.setName("vw");
    fwKinOdom.setName("fwKinOdom");
    posController.setName("posController");
    invKin.setName("invKin");
    piController.setName("PI Controller");
    invMotMod.setName("invMotMod");
    UM.setName("UM");
    Mwl.setName("Mwl");
    Mwr.setName("Mwr");

    // Name all signals
    Ewl.getOut().getSignal().setName("Left wheel position [m]");
    Ewr.getOut().getSignal().setName("Right wheel position [m]");
    KFwl.getX(0).getSignal().setName("Observed left wheel position [m]");
    KFwl.getX(1).getSignal().setName("Observed left wheel velocity [m/s]");
    KFwl.getX(2).getSignal().setName("Observed left motor current [A]");
    KFwl.getX(3).getSignal().setName("Observed left wheel acceleration offset [m/s²]");
    KFwr.getX(0).getSignal().setName("Observed right wheel position [m]");
    KFwr.getX(1).getSignal().setName("Observed right wheel velocity [m/s]");
    KFwr.getX(2).getSignal().setName("Observed right motor current [A]");
    KFwr.getX(3).getSignal().setName("Observed right wheel acceleration offset [m/s²]");
    vw.getOut().getSignal().setName("Observed wheel velocities [m/s]");
    UM.getOut(0).getSignal().setName("Left motor voltage setpoint [V]");
    UM.getOut(0).getSignal().setName("Right motor voltage setpoint [V]");

    // Connect signals
    KFwl.getY(0).connect(Ewl.getOut());
    KFwr.getY(0).connect(Ewr.getOut());
    vw.getIn(0).connect(KFwl.getX(1));
    vw.getIn(1).connect(KFwr.getX(1));
    fwKinOdom.getIn().connect(vw.getOut());
    posController.getIn().connect(fwKinOdom.getOutGrT());
    invKin.getInGvT().connect(posController.getOut());
    invKin.getInPhi().connect(fwKinOdom.getOutphi());
    piController.getInqds().connect(invKin.getOut());
    piController.getInqd().connect(vw.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    UM.getIn().connect(invMotMod.getOutU());
    KFwl.getU(0).connect(UM.getOut(0));
    KFwr.getU(0).connect(UM.getOut(1));
    Mwl.getIn().connect(UM.getOut(0));
    Mwr.getIn().connect(UM.getOut(1));

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(KFwl.correct);
    timedomain.addBlock(KFwr.correct);
    timedomain.addBlock(vw);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(posController);
    timedomain.addBlock(invKin);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(UM);
    timedomain.addBlock(KFwl.predict);
    timedomain.addBlock(KFwr.predict);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}