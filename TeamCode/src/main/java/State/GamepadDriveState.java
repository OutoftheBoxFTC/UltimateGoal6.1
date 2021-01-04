package State;

import com.qualcomm.robotcore.hardware.*;

import Hardware.Packets.*;

import MathUtils.*;

/**
 * Drives the robot using the given gamepad. Acts as a drive state
 */

public class GamepadDriveState extends VelocityDriveState {
    Gamepad gamepad;

    public GamepadDriveState(StateMachine stateMachine, Gamepad gamepad){
        super(stateMachine);
        this.gamepad = gamepad;
    }

    @Override
    public Vector3 getVelocities() {
        return new Vector3(gamepad.left_stick_x, gamepad.left_stick_y, -gamepad.right_stick_x);
    }

    @Override
    public void update(SensorData sensorData, HardwareData hardwareData) {

    }
}
