package State.EventSystem;

import State.StateMachine;

public class LinearTriggerBuilder {
    private StateMachine stateMachine;
    private OpmodeVariables opmodeVariables;
    private String orderName;
    private int order;
    private boolean disableOnEndTrigger;
    public LinearTriggerBuilder(StateMachine stateMachine, OpmodeVariables opmodeVariables, String orderName){
        this.stateMachine = stateMachine;
        this.opmodeVariables = opmodeVariables;
        this.orderName = orderName;
        order = 1;
        disableOnEndTrigger = false;
    }

    public LinearTriggerBuilder setOrder(int order) {
        this.order = order;
        return this;
    }

    public LinearTriggerBuilder disableOnEndTrigger(){
        disableOnEndTrigger = true;
        return this;
    }

    public LinearTrigger build(){
        LinearTrigger out = new LinearTrigger(stateMachine, order, disableOnEndTrigger, opmodeVariables, orderName);
        order = 0;
        disableOnEndTrigger = false;
        return out;
    }
}
