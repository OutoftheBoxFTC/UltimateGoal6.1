package State.EventSystem;

import java.util.*;

import Hardware.Packets.*;
import State.*;

/**
 * Event System Trigger
 * Contains a list of logicstates and a drivestate that are triggered when trigger() returns true
 */

public abstract class EventSystemTrigger {
    ArrayList<String> states;
    String driveState;
    StateMachine stateMachine;

    public EventSystemTrigger(StateMachine stateMachine){
        states = new ArrayList<>();
        driveState = "";
        this.stateMachine = stateMachine;
    }

    public void addState(String state){
        states.add(state);
    }

    public void addDriveState(String state, DriveState driveState){
        this.driveState = state;
        stateMachine.appendDriveState(state, driveState);
    }

    public abstract boolean trigger(SensorData sensorData, HardwareData hardwareData);

    public void update(SensorData sensorData, HardwareData hardwareData){
        if(trigger(sensorData, hardwareData)){
            for(String state : states){
                stateMachine.activateLogic(state);
            }
            stateMachine.setActiveDriveState(driveState);
        }
    }
}
