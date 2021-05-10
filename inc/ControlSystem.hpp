#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include "customBlocks/PD_controller.hpp"
#include <eeros/control/D.hpp>
#include <eeros/control/Gain.hpp>
#include <eeros/control/Saturation.hpp>
#include <eeros/control/Sum.hpp>
#include <eeros/control/DeMux.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Mux<2> E, E_set;
    PDController<eeros::math::Vector2> controller;
    D<eeros::math::Vector2> d;
    Gain<eeros::math::Vector2> i, kM, i_inv, kM_inv, R;
    Saturation<eeros::math::Vector2> qdMax, QMax;
    Sum<2, eeros::math::Vector2> U;
    DeMux<2> M;
    PeripheralOutput<> M1, M2;
    
    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP