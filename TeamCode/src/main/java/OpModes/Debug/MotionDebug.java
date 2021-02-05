package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.ArrayList;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathUtils.*;
import MathUtils.Vector3;
import Motion.DriveToPoint.DriveToPoint;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.PurePursuit.PurePursuitBuilder;
import Motion.PurePursuitOptimized.PurePursuitOptimized;
import Motion.PurePursuitOptimized.PurePursuitOptimizedBuilder;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
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
        eventSystem.onStart("Drive", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(-.25, 31.75))
                .setSpeed(0.2)
                .complete());
        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position);
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
            }
        });
    }
}
