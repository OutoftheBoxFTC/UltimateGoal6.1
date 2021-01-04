package State;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;

public class DriveStateActivator extends LogicState {
    String driveState;

    public DriveStateActivator(StateMachine stateMachine, String driveState) {
        super(stateMachine);
        this.driveState = driveState;
    }

    @Override
    public void init(SensorData sensorData, HardwareData hardwareData) {
        stateMachine.setActiveDriveState(driveState);
        deactivateThis();
    }

    @Override
    public void update(SensorData sensorData, HardwareData hardwareData) {

    }
}
