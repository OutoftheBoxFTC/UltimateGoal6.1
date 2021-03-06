package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import OpModes.BasicOpmode;
import State.LogicState;
@TeleOp
public class FullDebug extends BasicOpmode {
    public FullDebug() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        eventSystem.onStart("Start", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Odo L", sensorData.getOdometryLeft());
                telemetry.addData("Odo R", sensorData.getOdometryRight());
                telemetry.addData("Odo A", sensorData.getOdometryAux());
                telemetry.addData("Shooter", hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity());
                telemetry.addData("Shooter Pos", hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getPosition());
                telemetry.addData("Lift", sensorData.getWobbleLift());
                telemetry.addData("FPS", fps);
                telemetry.addData("Hardware Overhead", sensorData.getFps());
                telemetry.addData("Backlog", sensorData.getBacklog());
            }
        });
    }
}
