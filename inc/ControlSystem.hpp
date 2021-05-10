#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include "customBlocks/PD_controller.hpp"
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    PDController<> controller;
    D<> d;
    Gain<> i, kM, i_inv, kM_inv, R;
    Saturation<> qdMax, QMax;
    Sum<> U;
    PeripheralOutput<> M1;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP