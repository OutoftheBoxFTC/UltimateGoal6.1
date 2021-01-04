package State;

import MathUtils.Vector4;

/**
 * DriveState is an extension of LogicState
 * Only one drivestate can be active at a time
 * getDriveVelocities should return motor velocities in the order Front Left, Front Right, Back Left, Back Right
 */

public abstract class DriveState extends LogicState {
    public DriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    public abstract Vector4 getDriveVelocities();
}
