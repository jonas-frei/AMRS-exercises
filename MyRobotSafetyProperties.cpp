#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
      slSystemOff("System is offline"),
      slSystemOn("System is online"),

      doSystemOn("Startup the system"),
      doSystemOff("Shutdown the system")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare critical outputs
    // ... = hal.getLogicOutput("...");

    // criticalOutputs = { ... };

    // Declare critical inputs
    // ... = eeros::hal::HAL::instance().getLogicInput(..., ...);

    // criticalInputs = { ... };

    // Add all safety levels
    addLevel(slSystemOff);
    addLevel(slSystemOn);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slSystemOn, kPublicEvent);
    slSystemOn.addEvent(doSystemOff, slSystemOff, kPublicEvent);

    // Add events to multiple safety levels
    // addEventToAllLevelsBetween(lowerLevel, upperLevel, event, targetLevel, kPublicEvent/kPrivateEvent);

    // Define input states and events for all levels
    //level.setInputActions({...});

    // Define output states and events for all levels
    // level.setOutputActions({ ... });

    // Define and add level functions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        eeros::Executor::stop();
    });

    slSystemOn.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.start();
    });

    // Define entry level
    setEntryLevel(slSystemOff);

    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(doSystemOff);
    });
}
