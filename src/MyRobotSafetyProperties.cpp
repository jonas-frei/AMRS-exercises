#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),

      slSystemOff("System is offline"),
      slShuttingDown("System shutting down"),
      slHalting("System halting"),
      slStartingUp("System starting up"),
      slEmergency("Emergency"),
      slEmergencyBraking("System halting"),
      slSystemOn("System is online"),
      slMotorPowerOn("Motors powered"),
      slSystemMoving("System moving"),

      abort("Abort"),
      shutdown("Shutdown"),
      doSystemOn("Do system on"),
      systemStarted("System started"),
      emergency("Emergency"),
      resetEmergency("Reset emergency"),
      powerOn("Power on"),
      powerOff("Power off"),
      startMoving("Start moving"),
      stopMoving("Stop moving"),
      motorsHalted("Motors halted")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    redLed = hal.getLogicOutput("onBoardLEDred");
    greenLed = hal.getLogicOutput("onBoardLEDgreen");

    criticalOutputs = {redLed, greenLed};

    // Declare and add critical inputs
    readyButton = hal.getLogicInput("onBoardButtonPause", true);

    criticalInputs = {readyButton};

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slHalting);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff, kPrivateEvent);
    slHalting.addEvent(motorsHalted, slShuttingDown, kPrivateEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPrivateEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPrivateEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPrivateEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergencyBraking, kPublicEvent);
    slSystemMoving.addEvent(abort, slHalting, kPublicEvent);

    // Add events to multiple safety levels
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    // Define input actions for all levels
    slSystemOff.setInputActions({ignore(readyButton)});
    slShuttingDown.setInputActions({ignore(readyButton)});
    slHalting.setInputActions({ignore(readyButton)});
    slStartingUp.setInputActions({ignore(readyButton)});
    slEmergency.setInputActions({ignore(readyButton)});
    slEmergencyBraking.setInputActions({ignore(readyButton)});
    slSystemOn.setInputActions({check(readyButton, false, powerOn)});
    slMotorPowerOn.setInputActions({ignore(readyButton)});
    slSystemMoving.setInputActions({ignore(readyButton)});

    // Define output actions for all levels
    slSystemOff.setOutputActions({set(redLed, true), set(greenLed, false)});
    slShuttingDown.setOutputActions({set(redLed, true), set(greenLed, false)});
    slHalting.setOutputActions({set(redLed, true), set(greenLed, false)});
    slStartingUp.setOutputActions({set(redLed, true), set(greenLed, false)});
    slEmergency.setOutputActions({set(redLed, true), set(greenLed, false)});
    slEmergencyBraking.setOutputActions({set(redLed, true), set(greenLed, false)});
    slSystemOn.setOutputActions({set(redLed, true), set(greenLed, true)});
    slMotorPowerOn.setOutputActions({set(redLed, false), set(greenLed, true)});
    slSystemMoving.setOutputActions({set(redLed, false), set(greenLed, true)});

    // Define and add level actions
    slSystemOff.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            eeros::Executor::stop();
            eeros::sequencer::Sequencer::instance().abort();
        });

    slShuttingDown.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            cs.timedomain.stop();
            privateContext->triggerEvent(shutdown);
        });

    slHalting.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            if (cs.vw.getOut().getSignal().getValue() <= 1e-6)
                privateContext->triggerEvent(motorsHalted);
        });

    slStartingUp.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            cs.timedomain.start();
            cs.fwKinOdom.enableIntegrators();
            privateContext->triggerEvent(systemStarted);
        });

    slEmergency.setLevelAction(
        [&, dt](SafetyContext *privateContext)
        {
            cs.piController.disableIntegrator();
            cs.posController.disable();
        });

    slEmergencyBraking.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            cs.piController.disableIntegrator();
            cs.posController.disable();
            if (cs.vw.getOut().getSignal().getValue() <= 1e-6)
                privateContext->triggerEvent(motorsHalted);
        });

    slSystemOn.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            cs.piController.disableIntegrator();
            cs.posController.disable();
        });

    slMotorPowerOn.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            cs.piController.enableIntegrator();
            cs.posController.enable();
            if (cs.invKin.getOut().getSignal().getValue() > 1e-3)
                privateContext->triggerEvent(startMoving);
        });

    slSystemMoving.setLevelAction(
        [&](SafetyContext *privateContext)
        {
            if (cs.invKin.getOut().getSignal().getValue() <= 1e-3)
                privateContext->triggerEvent(stopMoving);
        });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext)
                    { privateContext->triggerEvent(abort); });
}
