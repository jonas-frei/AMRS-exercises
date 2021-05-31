#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : Ewl("enc1"), Ewr("enc2"),
      fwKinOdom(AMRSC::ROB::B),
      posController(AMRSC::CONT::K1, AMRSC::CONT::K2, AMRSC::CONT::K3),
      invKin(AMRSC::ROB::B),
      piController(1.0 / dt, AMRSC::CONT::D, AMRSC::CONT::s, AMRSC::CONT::M, AMRSC::CONT::ILIMIT),
      invMotMod(AMRSC::MOT::QMAX, AMRSC::MOT::qdMAX, AMRSC::MOT::i, AMRSC::MOT::KM, AMRSC::MOT::R),
      Mwl("motor1"), Mwr("motor2"),
      traceq(10000),
      tracedq(10000),
      traceU(10000),
      lowPassFilter1(1, 0.2),
      lowPassFilter2(1, 0.2),

      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    Ewl.setName("Ewl");
    Ewr.setName("Ewr");
    mux.setName("mux");
    vw.setName("vw");
    fwKinOdom.setName("fwKinOdom");
    posController.setName("posController");
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
    deMux.getOut(0).getSignal().setName("Left motor voltage setpoint [V]");
    deMux.getOut(0).getSignal().setName("Right motor voltage setpoint [V]");

    // Connect signals
    mux.getIn(0).connect(Ewl.getOut());
    mux.getIn(1).connect(Ewr.getOut());
    vw.getIn().connect(mux.getOut());
    fwKinOdom.getIn().connect(vw.getOut());
    posController.getInGrR().connect(fwKinOdom.getOutGrR());
    posController.getInphi().connect(fwKinOdom.getOutphi());
    lowPassFilter1.getIn().connect(posController.getOutRvRx());
    lowPassFilter2.getIn().connect(posController.getOutomegaR());
    invKin.getIn(0).connect(lowPassFilter1.getOut());
    invKin.getIn(1).connect(lowPassFilter2.getOut());
    piController.getInqds().connect(invKin.getOut());
    piController.getInqd().connect(vw.getOut());
    invMotMod.getInQ().connect(piController.getOutQ());
    invMotMod.getInqd().connect(piController.getOutqd());
    deMux.getIn().connect(invMotMod.getOutU());
    Mwl.getIn().connect(deMux.getOut(0));
    Mwr.getIn().connect(deMux.getOut(1));
    traceq.getIn().connect(mux.getOut());
    tracedq.getIn().connect(vw.getOut());
    traceU.getIn().connect(invMotMod.getOutU());
    traceq.enable();
    tracedq.enable();
    traceU.enable();

    // Add blocks to timedomain
    timedomain.addBlock(Ewl);
    timedomain.addBlock(Ewr);
    timedomain.addBlock(mux);
    timedomain.addBlock(vw);
    timedomain.addBlock(fwKinOdom);
    timedomain.addBlock(posController);
    timedomain.addBlock(lowPassFilter1);
    timedomain.addBlock(lowPassFilter2);
    timedomain.addBlock(invKin);
    timedomain.addBlock(piController);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(deMux);
    timedomain.addBlock(Mwl);
    timedomain.addBlock(Mwr);
    timedomain.addBlock(traceq);
    timedomain.addBlock(tracedq);
    timedomain.addBlock(traceU);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}