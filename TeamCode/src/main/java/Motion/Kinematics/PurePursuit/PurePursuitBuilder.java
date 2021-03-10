package Motion.Kinematics.PurePursuit;

import java.util.ArrayList;

import State.StateMachine;
import MathSystems.*;

/**
 * Builds a Pure Pursuit class, returning a drive state that can be used in the statemachine
 */

public class PurePursuitBuilder {
    private StateMachine stateMachine;
    private Vector3 position, end;
    public ArrayList<Vector2> targets;
    private double power, radius, rotTarget;
    public PurePursuitBuilder(StateMachine stateMachine, Vector3 position){
        this.stateMachine = stateMachine;
        this.position = position;
        this.targets = new ArrayList<>();
        this.power = 1;
        this.radius = 5;
        this.rotTarget = 0;
        this.end = Vector3.ZERO();
    }

    public PurePursuitBuilder addTarget(Vector2 v){
        targets.add(v);
        return this;
    }

    public PurePursuitBuilder setSpeed(double speed){
        this.power = speed;
        return this;
    }

    public PurePursuitBuilder setRadius(double radius){
        this.radius = radius;
        return this;
    }

    public PurePursuitBuilder setAngle(double angle){
        this.rotTarget = angle;
        return this;
    }

    public PurePursuitBuilder setEnd(Vector3 end){
        this.end = end;
        return this;
    }

    public PurePursuit complete(){
        return new PurePursuit(stateMachine, position, power, radius, targets, rotTarget);
    }
}