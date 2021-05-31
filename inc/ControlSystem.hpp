#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>

#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/fwKinOdom.hpp"
#include "customBlocks/PositionController.hpp"
#include "customBlocks/InvKin.hpp"
#include "customBlocks/PIController.hpp"
#include "customBlocks/InvMotMod.hpp"
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>
#include "AMRSConstants.hpp"
#include <eeros/control/Trace.hpp>
#include "customBlocks/LowPassFilter.hpp"

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
    PositionController posController;
    InvKin invKin;
    PIController<eeros::math::Vector2> piController;
    InvMotMod<eeros::math::Vector2> invMotMod;
    DeMux<2> deMux;
    PeripheralOutput<> Mwl, Mwr;
    Trace<eeros::math::Vector2> traceq, tracedq, traceU;
    LowPassFilter<> lowPassFilter1, lowPassFilter2;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP