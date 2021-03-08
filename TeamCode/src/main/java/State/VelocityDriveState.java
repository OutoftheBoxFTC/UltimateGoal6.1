package State;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.RobotSystems.*;
import MathSystems.*;

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

    public static VelocityDriveState STOP(StateMachine stateMachine){
        return new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        };
    }
}
