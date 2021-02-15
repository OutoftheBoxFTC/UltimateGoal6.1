package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.UltimateGoalHardware;
import MathSystems.Vector4;
import OpModes.BasicOpmode;
import State.LogicState;
@TeleOp
public class RobotCheck extends BasicOpmode {
    public RobotCheck() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        eventSystem.onStart("Main", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setDriveMotors(new Vector4(gamepad1.a ? 1 : 0, gamepad1.b ? 1 : 0, gamepad1.y ? 1 : 0, gamepad1.x ? 1 : 0));
            }
        });
    }
}
