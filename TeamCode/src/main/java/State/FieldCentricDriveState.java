package State;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.RobotSystems.MecanumSystem;
import MathUtils.Vector2;
import MathUtils.Vector3;
import MathUtils.Vector4;

public abstract class FieldCentricDriveState extends DriveState {
    public FieldCentricDriveState(StateMachine stateMachine) {
        super(stateMachine);
    }

    @Override
    public Vector4 getDriveVelocities() {
        Vector4 velocities = getVelocities();
        Vector2 polar = MathUtils.MathUtils.toPolar(velocities.getA(), velocities.getB());
        Vector2 cartesian = MathUtils.MathUtils.toCartesian(polar.add(new Vector2(0, velocities.getD())));
        return MecanumSystem.translate(cartesian.toVector3(velocities.getC()));
    }

    public abstract Vector4 getVelocities();
}
