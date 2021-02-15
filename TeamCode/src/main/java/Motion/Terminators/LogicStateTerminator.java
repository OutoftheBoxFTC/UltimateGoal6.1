package Motion.Terminators;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import State.StateMachine;

public class LogicStateTerminator extends Terminator {
    StateMachine stateMachine;
    String stateName;

    public LogicStateTerminator(StateMachine stateMachine, String stateName){
        this.stateMachine = stateMachine;
        this.stateName = stateName;
    }

    @Override
    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
        return !stateMachine.logicStateActive(stateName);
    }
}
