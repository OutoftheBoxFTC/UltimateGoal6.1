package State.EventSystem;

import java.util.ArrayList;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Motion.Terminators.Terminator;
import State.LogicState;
import State.StateMachine;

public class LinearEventSystem{
    private StateMachine stateMachine;
    private ArrayList<String> logicStates;
    private ArrayList<Terminator> terminators;
    private int currentIndex = -1;
    private ENDTYPE endtype;
    private boolean run = true;
    public LinearEventSystem(StateMachine stateMachine, ENDTYPE endtype) {
        this.stateMachine = stateMachine;
        logicStates = new ArrayList<>();
        terminators = new ArrayList<>();
        this.endtype = endtype;
    }

    public void put(String stateName, Terminator terminator){
        logicStates.add(stateName);
        terminators.add(terminator);
    }

    public void update(SensorData sensorData, HardwareData hardwareData) {
        if(run) {
            if (currentIndex == -1) {
                if (logicStates.size() > 0) {
                    stateMachine.activateLogic(logicStates.get(0));
                    currentIndex++;
                }
            }
            if (terminators.get(currentIndex).shouldTerminate(sensorData, hardwareData)) {
                if (currentIndex >= logicStates.size()) {
                    if (endtype == ENDTYPE.CONTINUE_LAST) {
                        return;
                    }
                    if (endtype == ENDTYPE.STOP_ALL) {
                        stateMachine.deactivateState(logicStates.get(currentIndex));
                        run = false;
                    }
                    if(endtype == ENDTYPE.LOOP){
                        stateMachine.deactivateState(logicStates.get(currentIndex));
                        currentIndex = 0;
                        stateMachine.activateLogic(logicStates.get(currentIndex));
                    }
                } else {
                    stateMachine.deactivateState(logicStates.get(currentIndex));
                    currentIndex ++;
                    stateMachine.activateLogic(logicStates.get(currentIndex));
                }
            }
        }
    }

    public enum ENDTYPE{
        STOP_ALL,
        CONTINUE_LAST,
        LOOP;
    }
}
