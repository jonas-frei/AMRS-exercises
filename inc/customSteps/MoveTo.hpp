#ifndef MOVETO_HPP_
#define MOVETO_HPP_

#include <eeros/sequencer/Step.hpp>
#include <eeros/sequencer/Sequence.hpp>
#include "ControlSystem.hpp"

class MoveTo : public eeros::sequencer::Step
{
public:
    MoveTo(std::string name, eeros::sequencer::Sequence *caller, ControlSystem &cs)
        : eeros::sequencer::Step(name, caller),
          cs(cs)
    {
        log.info() << "Step created: " << name;
    }

    int operator()(double xT, double yT, double phiT)
    {
        this->xT = xT;
        this->yT = yT;
        this->phiT = phiT;
        return start();
    }

    int action()
    {
        cs.posController.setTarget(xT, yT, phiT);
        log.info() << "Target pose set to x: " << xT << ", y: " << yT << ", phi: " << phiT;
        log.info() << "Start moving.";
        return 0;
    }

    bool checkExitCondition()
    {
        if (cs.posController.getStatus())
            log.info() << "Target pose reached.";
        return cs.posController.getStatus();
    }

private:
    double xT, yT, phiT;
    ControlSystem &cs;
};

#endif // MOVETO_HPP_
