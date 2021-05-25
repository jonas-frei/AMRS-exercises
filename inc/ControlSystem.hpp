#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/InvMotMod.hpp"
#include "customBlocks/PIController.hpp"
#include <eeros/control/PeripheralOutput.hpp>
#include "AMRSConstants.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> Ewl, Ewr;
    D<> Dwl, Dwr;
    Constant<> myConstant, myConstant2;
    PIController<> piController;
    InvMotMod<> invMotMod;
    PeripheralOutput<> Mwl, Mwr;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP