package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.Vector3;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
@Disabled
public class MotionDebug extends BasicOpmode {
    Vector3 position, velocity;
    Odometer odometer;
    public MotionDebug() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Odometer", odometer);

        eventSystem.onStart("Drive", new GamepadDriveState(stateMachine, gamepad1));

        eventSystem.onStart("Logging", new LogicState(stateMachine) {
            double maxX, maxY, maxR;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                this.maxX = Math.max(maxX, Math.abs(velocity.getA()));
                this.maxY = Math.max(maxY, Math.abs(velocity.getB()));
                this.maxR = Math.max(maxR, Math.toDegrees(Math.abs(velocity.getC())));
                telemetry.addData("MaxX", maxX);
                telemetry.addData("MaxY", maxY);
                telemetry.addData("MaxR", maxR);
            }
        });



        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
                RobotLog.ii("Vel", velocity.toString());
            }
        });
    }
}
