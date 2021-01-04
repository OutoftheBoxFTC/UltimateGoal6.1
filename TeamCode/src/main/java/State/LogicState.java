package State;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;

/**
 * LogicStates are the cornerstone of the statemachine system
 * A logic state is a self contained group of code that is executed every frame
 * init is called when the state is first activated
 * update calls every frame the logic state is active
 * onStop is called when the logic state is stopped
 */

public abstract class LogicState {
    String name;
    StateMachine stateMachine;

    public LogicState(StateMachine stateMachine){
        this.stateMachine = stateMachine;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void init(SensorData sensorData, HardwareData hardwareData){

    }

    public void deactivateThis(){
        stateMachine.deactivateState(name);
    }

    public void onStop(SensorData sensorData, HardwareData hardwareData){

    }

    public abstract void update(SensorData sensorData, HardwareData hardwareData);

    @Override
    public String toString() {
        return "LogicState{" +
                "name='" + name + '\'' +
                '}';
    }
}
