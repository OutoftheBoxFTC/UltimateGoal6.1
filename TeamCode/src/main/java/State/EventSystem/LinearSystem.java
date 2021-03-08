package State.EventSystem;

import java.util.ArrayList;
import java.util.HashMap;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Motion.Terminators.Terminator;
import State.LogicState;
import State.StateMachine;

public class LinearSystem extends LogicState {
    private StateMachine stateMachine;
    private HashMap<String, Terminator> states;
    private ArrayList<String> statesList;
    private String endState;
    private int stateIdx;
    public LinearSystem(StateMachine stateMachine, HashMap<String, Terminator> states, String endState) {
        super(stateMachine);
        this.stateMachine = stateMachine;
        this.states = states;
        this.endState = endState;
        statesList = new ArrayList<>();
        statesList.addAll(states.keySet());
        stateIdx = 0;
    }


    @Override
    public void update(SensorData sensorData, HardwareData hardwareData) {
        if(stateIdx < statesList.size()){
            if(!stateMachine.logicStateActive(endState)){
                stateMachine.activateLogic(endState);
            }
            if(states.get(statesList.get(stateIdx)).shouldTerminate(sensorData, hardwareData)){
                stateIdx ++;
            }
        }else{
            if(statesList.get(stateIdx) != null && !stateMachine.logicStateActive(endState)){
                stateMachine.activateLogic(endState);
            }
        }
    }
}
