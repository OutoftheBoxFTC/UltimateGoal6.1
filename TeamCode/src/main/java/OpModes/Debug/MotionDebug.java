package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.*;
import MathSystems.Vector3;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveStateActivator;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
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

        Path p = new PathBuilder(new Vector3(0, 0, 0))
                .bezierSplineTo(new Vector2(30, 30), new Vector2(0, 30))
                .complete();

        CrosstrackBuilder b = new CrosstrackBuilder(stateMachine, position);

        eventSystem.onStart("Drive", b.follow(p));


        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
                //RobotLog.ii("Vel", velocity.toString());
            }
        });
    }
}
