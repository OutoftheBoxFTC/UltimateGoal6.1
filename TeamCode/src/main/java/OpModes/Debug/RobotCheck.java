package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
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
                hardwareData.setIntakeRelease(gamepad1.a ? RobotConstants.UltimateGoal.RELEASE_INTAKE : RobotConstants.UltimateGoal.HOLD_INTAKE);
            }
        });
    }
}
