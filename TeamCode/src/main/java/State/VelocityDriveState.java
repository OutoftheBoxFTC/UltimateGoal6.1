package State;

import Hardware.RobotSystems.*;
import MathUtils.*;

/**
 * VelocityDriveState is a drive state that takes in a vector3 (strafe, forward, rotation) speeds and automatically converts them to motor powers
 */

public abstract class VelocityDriveState extends DriveState{

    public VelocityDriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public Vector4 getDriveVelocities() {
        return MecanumSystem.translate(getVelocities());
    }

    public abstract Vector3 getVelocities();
}
