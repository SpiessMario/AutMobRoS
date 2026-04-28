#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
    
      slSystemOff("System is offline"),
      slShuttingDown("System is shutting down"),
      slBraking("System is braking"),
      slStartingUp("System is starting up"),
      slEmergency("System has an emergency"),
      slEmergencyBraking("System is braking because of an emergancy"),
      slSystemOn("System is online"),
      slMotorPowerOn("System is online and motors are enabled/powered"),
      slSystemMoving("System is in motion"),

      doSystemOn("Startup the system"),
      doSystemOff("Shut down the system"),
      systemStarted("System if finnished booting (ready)"),
      powerOn("Enable the motors"),
      powerOff("Disable the motors"),
      startMoving("Set robot in motion"),
      stopMoving("Bring the robot to a stop"),
      emergency("An emergancy has occured"),
      resetEmergency("Emergency is handled. There is no more emergency"),
      motorsHalted("Motors stoped moving"),
      abort("Preparing for/Starting system shut down")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    criticalOutputs = { greenLED, redLED };

    // Declare and add critical inputs
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause");
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode");

    criticalInputs = { buttonPause, buttonMode };

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slBraking);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBraking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(doSystemOff, slSystemOff, kPublicEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPublicEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPublicEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPublicEvent);
    slEmergencyBraking.addEvent(motorsHalted, slEmergency, kPublicEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergencyBraking, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);   

    // Add events to multiple safety levels
    // addEventToAllLevelsBetween(lowerLevel, upperLevel, event, targetLevel, kPublicEvent/kPrivateEvent);
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    // Define input actions for all levels
    slSystemOff.setInputActions({           ignore(buttonPause),                    ignore(buttonMode) });
    slShuttingDown.setInputActions({        ignore(buttonPause),                    ignore(buttonMode) });
    slBraking.setInputActions({             ignore(buttonPause),                    ignore(buttonMode) });
    slStartingUp.setInputActions({           ignore(buttonPause),                    ignore(buttonMode) });
    slEmergency.setInputActions({           ignore(buttonPause),                    check(buttonMode, false, resetEmergency) });
    slEmergencyBraking.setInputActions({    ignore(buttonPause),                    ignore(buttonMode) });
    slSystemOn.setInputActions({            check(buttonPause, false, emergency),   ignore(buttonMode) });
    slMotorPowerOn.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });
    slSystemMoving.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });

    // Define output actions for all levels
    slSystemOff.setOutputActions({           set(greenLED, false),   set(redLED, false) });
    slShuttingDown.setOutputActions({        set(greenLED, false),   set(redLED, true) });
    slBraking.setOutputActions({             set(greenLED, false),   set(redLED, true) });
    slStartingUp.setOutputActions({          set(greenLED, true),    set(redLED, false) });
    slEmergency.setOutputActions({           set(greenLED, true),    set(redLED, true) });
    slEmergencyBraking.setOutputActions({    set(greenLED, true),    set(redLED, true) });
    slSystemOn.setOutputActions({            set(greenLED, true),    set(redLED, false) });
    slMotorPowerOn.setOutputActions({        set(greenLED, true),    set(redLED, false) });
    slSystemMoving.setOutputActions({        set(greenLED, true),    set(redLED, false) });

    // Define and add level actions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        eeros::Executor::stop();
    });

    slShuttingDown.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        privateContext->triggerEvent(doSystemOff);
    });

    slBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing sill
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&](SafetyContext *privateContext) {
        
    });

    slEmergencyBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemOn.getNofActivations()*dt >= 1)   // wait 1 sec
        {
            privateContext->triggerEvent(powerOn);
        }
    });

    slMotorPowerOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slMotorPowerOn.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(startMoving);
        }
    });

    slSystemMoving.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemMoving.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(stopMoving);
        }
    });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}
