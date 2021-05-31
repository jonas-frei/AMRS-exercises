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
#include "customBlocks/KalmanFilter.hpp"

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> Ewl, Ewr;
    KalmanFilter<1, 1, 4, 1> KFwl, KFwr;
    Mux<2> vw;
    FwKinOdom fwKinOdom;
    PositionController posController;
    LowPassFilter<> lowPassRvRx, lowPassOmegaR;
    InvKin invKin;
    PIController<eeros::math::Vector2> piController;
    InvMotMod<eeros::math::Vector2> invMotMod;
    DeMux<2> UM;
    PeripheralOutput<> Mwl, Mwr;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP