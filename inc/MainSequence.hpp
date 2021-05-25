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
        while (eeros::sequencer::Sequencer::running && ss.getCurrentLevel() < sp.slSystemOn)
            ; // Wait for safety system to get into slSystemOn
            cs.piController.enableIntegrator();
            cs.fwKinOdom.enableIntegrators();
        while (eeros::sequencer::Sequencer::running)
        {
            log.info() << cs.fwKinOdom.getOutGrR().getSignal();
            log.info() << cs.fwKinOdom.getOutphi().getSignal();
            sleep(1.0);
            //log.info() << cs.Ewl.getOut().getSignal() << " " << cs.Ewr.getOut().getSignal();
        }
        return 0;
    }

private:
    eeros::safety::SafetySystem &ss;
    ControlSystem &cs;
    MyRobotSafetyProperties &sp;

    eeros::sequencer::Wait sleep;
};

#endif // MAINSEQUENCE_HPP_