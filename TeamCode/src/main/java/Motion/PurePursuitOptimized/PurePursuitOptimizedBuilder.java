package Motion.PurePursuitOptimized;

import java.util.ArrayList;

import MathUtils.*;
import State.StateMachine;

/**
 * Builds an optimized pure pursuit class, returning a drive state that can be used in the state machine
 */

public class PurePursuitOptimizedBuilder {
    private StateMachine stateMachine;
    private Vector3 position;
    public ArrayList<Vector2> targets;
    private double power, radius, rotTarget;
    public PurePursuitOptimizedBuilder(StateMachine stateMachine, Vector3 position){
        this.stateMachine = stateMachine;
        this.position = position;
        this.targets = new ArrayList<>();
        this.power = 1;
        this.radius = 5;
        this.rotTarget = 0;
    }

    public PurePursuitOptimizedBuilder addTarget(Vector2 v){
        targets.add(v);
        return this;
    }

    public PurePursuitOptimizedBuilder setSpeed(double speed){
        this.power = speed;
        return this;
    }

    public PurePursuitOptimizedBuilder setRadius(double radius){
        this.radius = radius;
        return this;
    }

    public PurePursuitOptimizedBuilder setApproachAngle(double angle){
        this.rotTarget = angle;
        return this;
    }

    public PurePursuitOptimized complete(){
        return new PurePursuitOptimized(stateMachine, position, power, radius, targets, rotTarget);
    }
}
