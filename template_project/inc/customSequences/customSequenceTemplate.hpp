#ifndef CUSTOMSEQUENCETEMPLATE_HPP_
#define CUSTOMSEQUENCETEMPLATE_HPP_

#include <eeros/sequencer/Sequence.hpp>

class CustomSequenceName : public eeros::sequencer::Sequence
{
public:
    CustomSequenceName(std::string name, eeros::sequencer::Sequence *caller)
        : eeros::sequencer::Sequence(name, caller, true)
    {
        log.info() << "Sequence created: " << name;
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

#endif // CUSTOMSEQUENCETEMPLATE_HPP_
