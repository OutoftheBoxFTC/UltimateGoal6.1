package OpModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathUtils.Vector3;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.*;
import State.GamepadDriveState;
import State.LogicState;

/**
 * Example Teleop class
 * Demonstrates the usage of
 * Hardware register and enable functions
 * GamepadDriveState
 * Event System onStart trigger
 */
@TeleOp
public class ExampleTeleop extends BasicOpmode {
    Vector3 position, velocity, simplePos, simpleVel;
    Odometer odometer;
    public ExampleTeleop() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        hardware.deRegisterDevice(Hardware.HardwareDevices.GYRO);
        hardware.disableDevice(Hardware.HardwareDevices.GYRO);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        simplePos = Vector3.ZERO();
        simpleVel = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Drive", new GamepadDriveState(stateMachine, gamepad1));
        eventSystem.onStart("Odometer", odometer);
        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position.getVector2().toVector3(Math.toDegrees(position.getC())));
                telemetry.addData("Angle", sensorData.getGyro());
                telemetry.addData("FPS", fps);
                telemetry.addData("HardwareFPS", sensorData.getFps());
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
                telemetry.addData("Difference (R-L)", sensorData.getOdometryRight() - sensorData.getOdometryLeft());
            }
        });
    }
}
