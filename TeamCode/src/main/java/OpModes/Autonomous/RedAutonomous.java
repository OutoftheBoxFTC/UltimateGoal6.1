package OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.HashMap;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.*;
import MathSystems.Vector3;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.PurePursuitOptimized.PurePursuitOptimizedBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.TimeTerminator;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.DriveStateActivator;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
@Autonomous
public class RedAutonomous extends BasicOpmode {
    Vector3 position, velocity;
    Odometer odometer;
    public RedAutonomous() {
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

        opmodeVariables.integers.put("count", 0);

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("Drive To Left Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(-2, 45.75))
                .setSpeed(0.7)
                .complete());
        driveStates.put("Drive To Centre Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(6, 45.75))
                .setSpeed(0.7)
                .complete());
        driveStates.put("Drive to Right Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(14, 45.75))
                .setSpeed(0.7)
                .complete());
        driveStates.put("Fourth Movement", new PurePursuitOptimizedBuilder(stateMachine, position)
                .addTarget(new Vector2(-15, 80))
                .addTarget(new Vector2(-20, 30))
                .addTarget(new Vector2(-10, 70))
                .setSpeed(0.4)
                .complete());
        driveStates.put("Test Movement", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, 10))
                .setSpeed(0.5)
                .complete());

        driveStates.put("Stop", new DriveState(stateMachine) {
            @Override
            public Vector4 getDriveVelocities() {
                return Vector4.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        stateMachine.appendDriveStates(driveStates);

        HashMap<String, LogicState> autoStates = new HashMap<>();
        autoStates.put("Left Powershot", new DriveStateActivator(stateMachine, "Drive To Left Powershot"));
        autoStates.put("Centre Powershot", new DriveStateActivator(stateMachine, "Drive To Centre Powershot"));
        autoStates.put("Right Powershot", new DriveStateActivator(stateMachine, "Drive To Right Powershot"));
        autoStates.put("Fourth", new DriveStateActivator(stateMachine, "Fourth Movement"));
        autoStates.put("Test", new DriveStateActivator(stateMachine, "Test Movement"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop"));
        stateMachine.appendLogicStates(autoStates);

        final LinearEventSystem linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);
        linearSystem.put("Left Powershot", new OrientationTerminator(position, new Vector3(-2, 45.75, 0), 0.3));
        linearSystem.put("End", new TimeTerminator(300));
        linearSystem.put("Centre Powershot", new OrientationTerminator(position, new Vector3(6, 45.75, 0), 0.3));
        linearSystem.put("End2", new TimeTerminator(300));
        linearSystem.put("Right Powershot", new OrientationTerminator(position, new Vector3(14, 45.75, 0), 0.3));
        //linearSystem.put("Third", new OrientationTerminator(position, new Vector3(-15, 80, 0), 5));
        //linearSystem.put("Fourth", new OrientationTerminator(position, new Vector3(-10, 70, 0), 5));
        //linearSystem.put("End", Terminator.nullTerminator());

        eventSystem.onStart("Main", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.update(sensorData, hardwareData);
            }
        });

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
