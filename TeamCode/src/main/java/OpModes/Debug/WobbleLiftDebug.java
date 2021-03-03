package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import OpModes.BasicOpmode;
import State.LogicState;
@TeleOp
public class WobbleLiftDebug extends BasicOpmode {
    public WobbleLiftDebug() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        eventSystem.onStart("Start", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Wobble", sensorData.getWobbleLift());
                hardwareData.setWobbleLift(gamepad1.dpad_up ? 0.5 : (gamepad1.dpad_down ? -0.5 : 0));
            }
        });
    }
}