package State;

import java.util.*;

import Hardware.Packets.*;

import MathUtils.*;

/**
 * The statemachine is the main system for executing code
 * Logic States are run asynchronously to each other, on the same thread
 * Any number of logic states can be added and activated, so you should separate code into separate states for readability
 * States can also be enabled and disabled, allowing for timers and other systems to be run cleanly and asynchronously
 */

public class StateMachine {
    HashMap<String, LogicState> queriedLogicStates, activeLogicStates, logicStates, deactivatedLogicStates;
    HashMap<String, Long> delayedActivations;
    ArrayList<String> driveStates;
    String activeDriveState;
    DriveState driveState;

    public StateMachine(){
        queriedLogicStates = new HashMap<>();
        activeLogicStates = new HashMap<>();
        driveStates = new ArrayList<>();
        logicStates = new HashMap<>();
        delayedActivations = new HashMap<>();
        deactivatedLogicStates = new HashMap<>();
        activeDriveState = "";
    }

    public void update(SensorData sensors, HardwareData hardware){
        for(String state : queriedLogicStates.keySet()){
            queriedLogicStates.get(state).init(sensors, hardware);
        }
        activeLogicStates.putAll(queriedLogicStates);
        queriedLogicStates.clear();
        for(String state : activeLogicStates.keySet()){
            activeLogicStates.get(state).update(sensors, hardware);
        }
        for(String state : delayedActivations.keySet()){
            if(System.nanoTime() >= delayedActivations.get(state)){
                activateLogic(state);
                delayedActivations.remove(state);
            }
        }
        for(String state : deactivatedLogicStates.keySet()){
            if(activeLogicStates.containsKey(state)){
                activeLogicStates.get(state).onStop(sensors, hardware);
                activeLogicStates.remove(state);
            }
        }
        deactivatedLogicStates.clear();
    }

    public boolean containsState(String state){
        return logicStates.containsKey(state);
    }

    public boolean logicStateActive(String state){
        return activeLogicStates.containsKey(state) || queriedLogicStates.containsKey(state);
    }

    public boolean driveStateActive(String state){
        return activeDriveState.equals(state);
    }

    public void appendLogicState(String name, LogicState state){
        state.setName(name);
        logicStates.put(name, state);
    }

    public void appendLogicStates(HashMap<String, LogicState> map){
        for(String state : map.keySet()){
            map.get(state).setName(state);
        }
        logicStates.putAll(map);
    }

    public void appendDriveState(String name, DriveState state){
        state.setName(name);
        logicStates.put(name, state);
        driveStates.add(name);
    }

    public void appendDriveStates(HashMap<String, DriveState> map){
        for(String state : map.keySet()){
            map.get(state).setName(state);
        }
        driveStates.addAll(map.keySet());
        logicStates.putAll(map);
    }

    public boolean activateLogic(String state){
        if(logicStates.containsKey(state) && (!activeLogicStates.containsKey(state))){
            queriedLogicStates.put(state, logicStates.get(state));
            return true;
        }
        return false;
    }

    public boolean setActiveDriveState(String state){
        if(driveStates.contains(state) && (!activeDriveState.equals(state))){
            queriedLogicStates.put(state, logicStates.get(state));
            activeDriveState = state;
            return true;
        }
        return false;
    }

    public boolean queryActivation(String state, long delay){
        if(logicStates.containsKey(state)){
            delayedActivations.put(state, System.nanoTime() + MathUtils.millisToNano(delay));
            return true;
        }
        return false;
    }

    public boolean deactivateState(String state){
        if(activeLogicStates.containsKey(state)){
            deactivatedLogicStates.put(state, activeLogicStates.get(state));
            return true;
        }
        return false;
    }

    public Vector4 getDriveVelocities(){
        if(logicStates.containsKey(activeDriveState)){
            return ((DriveState) logicStates.get(activeDriveState)).getDriveVelocities();
        }
        return Vector4.ZERO();
    }

    @Override
    public String toString() {
        String s = "Active States: ";
        for(String str : activeLogicStates.keySet()){
            s += str + " ";
        }
        s += "Active Drive State: " + activeDriveState;
        return s;
    }
}
