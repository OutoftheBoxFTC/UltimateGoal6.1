package State.EventSystem;

import java.util.HashMap;

import Motion.Terminators.Terminator;
import Motion.Terminators.TrueTimeTerminator;
import State.StateMachine;

public class LinearSystemBuilder {
    private StateMachine stateMachine;
    private HashMap<String, Terminator> states;
    public LinearSystemBuilder(StateMachine stateMachine){
        this.stateMachine = stateMachine;
        states = new HashMap<>();
    }

    public LinearSystemBuilder async(String state){
        states.put(state, Terminator.trueTerminator());
        return this;
    }

    public LinearSystemBuilder blocking(String state, Terminator endCondition){
        states.put(state, endCondition);
        return this;
    }

    public LinearSystemBuilder delay(long delay){
        states.put(null, new TrueTimeTerminator(delay));
        return this;
    }

    public LinearSystem complete(String endState){
        return new LinearSystem(stateMachine, states, endState);
    }
}
