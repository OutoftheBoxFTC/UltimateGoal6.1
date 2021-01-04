package Odometry;

import MathUtils.Vector3;
import State.LogicState;
import State.StateMachine;

/**
 * Basic Odometer Class
 * The Odometer has two Vector3s, a position and velocity variables
 * Both variables are updated via the set method, so the two variables can be accessed outside the odometer for position and velocity data
 */

public abstract class Odometer extends LogicState {
    Vector3 position, velocity;
    public Odometer(StateMachine stateMachine, Vector3 position, Vector3 velocity) {
        super(stateMachine);
        this.position = position;
        this.velocity = velocity;
    }
}
