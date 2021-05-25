#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/Constant.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/fwKinOdom.hpp"
#include "customBlocks/InvMotMod.hpp"
#include "customBlocks/PIController.hpp"
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include "AMRSConstants.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> Ewl, Ewr;
    Mux<2> mux;
    D<eeros::math::Vector2> vw;
    FwKinOdom fwKinOdom;
    Constant<eeros::math::Vector2> myConstant;
    PIController<eeros::math::Vector2> piController;
    InvMotMod<eeros::math::Vector2> invMotMod;
    DeMux<2> deMux;
    PeripheralOutput<> Mwl, Mwr;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP