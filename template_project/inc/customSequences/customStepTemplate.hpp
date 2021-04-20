#ifndef CUSTOMSTEPTEMPLATE_HPP_
#define CUSTOMSTEPTEMPLATE_HPP_

#include <eeros/sequencer/Step.hpp>

class CustomStepName : public eeros::sequencer::Step
{
public:
    CustomStepName(std::string name, eeros::sequencer::Sequence *caller)
        : eeros::sequencer::Step(name, caller)
    {
        log.info() << "Step created: " << name;
    }

    bool checkPreCondition()
    {
        return true;
    }

    int operator()()
    {
        return start();
    }

    int action()
    {
        return 0;
    }

    bool checkExitCondition()
    {
        return true;
    }

private:
};

#endif // CUSTOMSTEPTEMPLATE_HPP_
