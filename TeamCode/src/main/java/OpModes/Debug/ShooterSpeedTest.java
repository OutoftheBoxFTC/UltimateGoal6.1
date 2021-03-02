package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import OpModes.BasicOpmode;
import State.LogicState;
@TeleOp
public class ShooterSpeedTest extends BasicOpmode {
    public ShooterSpeedTest() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.enableDevice(Hardware.HardwareDevices.SHOOTER);
        hardware.registerDevice(Hardware.HardwareDevices.SHOOTER);
        eventSystem.onStart("Main", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooter(gamepad1.right_trigger - gamepad1.left_trigger);
                telemetry.addData("Shooter", hardwareData.getShooter());
                telemetry.addData("Left Shooter", hardware.getSmartDevices().get("Shooter Left", SmartMotor.class).getVelocity());
                telemetry.addData("Right Shooter", hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity());
            }
        });
    }
}
