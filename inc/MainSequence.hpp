#ifndef MAINSEQUENCE_HPP_
#define MAINSEQUENCE_HPP_

#include <eeros/sequencer/Sequencer.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include "MyRobotSafetyProperties.hpp"
#include "ControlSystem.hpp"
#include <eeros/sequencer/Wait.hpp>

class MainSequence : public eeros::sequencer::Sequence
{
public:
    MainSequence(std::string name, eeros::sequencer::Sequencer &seq,
                 eeros::safety::SafetySystem &ss,
                 MyRobotSafetyProperties &sp, ControlSystem &cs)
        : eeros::sequencer::Sequence(name, seq),
          ss(ss),
          sp(sp),
          cs(cs),

          sleep("Sleep", this)
    {
        log.info() << "Sequence created: " << name;
    }

    int action()
    {
        while (eeros::sequencer::Sequencer::running && ss.getCurrentLevel() < sp.slMotorPowerOn)
            ; // Wait for safety system to get into slMotorPowerOn
        /*cs.piController.enableIntegrator();
        cs.fwKinOdom.enableIntegrators();
        cs.posController.enable();*/
        sleep(1.0);
        cs.posController.setTarget(0.5, 0.0, 0.0);
        while (eeros::sequencer::Sequencer::running && !cs.posController.getStatus())
        {
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutphi().getSignal();
            sleep(1.0);
            //log.info() << cs.Ewl.getOut().getSignal() << " " << cs.Ewr.getOut().getSignal();
        }
        cs.posController.setTarget(0.5, 0.5, M_PI/2);
        while (eeros::sequencer::Sequencer::running && !cs.posController.getStatus())
        {
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutphi().getSignal();
            sleep(1.0);
            //log.info() << cs.Ewl.getOut().getSignal() << " " << cs.Ewr.getOut().getSignal();
        }
        cs.posController.setTarget(0.0, 0.5, M_PI);
        while (eeros::sequencer::Sequencer::running && !cs.posController.getStatus())
        {
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutphi().getSignal();
            sleep(1.0);
            //log.info() << cs.Ewl.getOut().getSignal() << " " << cs.Ewr.getOut().getSignal();
        }
        cs.posController.setTarget(0.0, 0.0, 0.0);
        while (eeros::sequencer::Sequencer::running && !cs.posController.getStatus())
        {
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutphi().getSignal();
            sleep(1.0);
            //log.info() << cs.Ewl.getOut().getSignal() << " " << cs.Ewr.getOut().getSignal();
        }
        ss.triggerEvent(sp.abort);
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    MyRobotSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
};

#endif // MAINSEQUENCE_HPP_