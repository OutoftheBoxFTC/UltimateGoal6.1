package State;

import Hardware.RobotSystems.MecanumSystem;
import MathSystems.Vector2;
import MathSystems.Vector4;

public abstract class FieldCentricDriveState extends DriveState {
    public FieldCentricDriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public Vector4 getDriveVelocities() {
        Vector4 velocities = getVelocities();
        Vector2 polar = Math.MathUtils.toPolar(velocities.getA(), velocities.getB());
        Vector2 cartesian = Math.MathUtils.toCartesian(polar.add(new Vector2(0, velocities.getD())));
        return MecanumSystem.translate(cartesian.toVector3(velocities.getC()));
    }

    public abstract Vector4 getVelocities();
}
