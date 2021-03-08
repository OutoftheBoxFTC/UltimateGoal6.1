package OpModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.*;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.*;
import Motion.Kinematics.DriveToPoint.DriveToPointBuilder;
import Motion.Kinematics.PurePursuitOptimized.PurePursuitOptimizedBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.Terminator;
import Odometry.*;
import OpModes.*;
import State.*;
import State.EventSystem.LinearEventSystem;

@Autonomous
public class ExampleAutonomous extends BasicOpmode {
    Vector3 position, velocity;
    Odometer odometer;
    public ExampleAutonomous() {
        super(new SkystoneHardware());
    }

    @Override
    public void setup() {
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Odometer", odometer);

        opmodeVariables.integers.put("count", 0);

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("First Movement", new PurePursuitOptimizedBuilder(stateMachine, position)
                .addTarget(new Vector2(-10, 30))
                .addTarget(new Vector2(-20, 30))
                .addTarget(new Vector2(-20, 50))
                .setSpeed(0.4)
                .complete());
        driveStates.put("Second Movement", new PurePursuitOptimizedBuilder(stateMachine, position)
                .addTarget(new Vector2(-20, 50))
                .addTarget(new Vector2(-10, 90))
                .addTarget(new Vector2(-15, 20))
                .setSpeed(0.4)
                .complete());
        driveStates.put("Third Movement", new PurePursuitOptimizedBuilder(stateMachine, position)
                .addTarget(new Vector2(-15, 20))
                .addTarget(new Vector2(-15, 80))
                .setSpeed(0.4)
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
        autoStates.put("First", new DriveStateActivator(stateMachine, "First Movement"));
        autoStates.put("Second", new DriveStateActivator(stateMachine, "Second Movement"));
        autoStates.put("Third", new DriveStateActivator(stateMachine, "Third Movement"));
        autoStates.put("Fourth", new DriveStateActivator(stateMachine, "Fourth Movement"));
        autoStates.put("Test", new DriveStateActivator(stateMachine, "Test Movement"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        stateMachine.appendLogicStates(autoStates);

        final LinearEventSystem linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);
        linearSystem.put("Test", new OrientationTerminator(position, new Vector3(0, 10, 0), 1, 0.5));
        linearSystem.put("First", new OrientationTerminator(position, new Vector3(-20, 50, 0), 5, 0.5));
        linearSystem.put("Second", new OrientationTerminator(position, new Vector3(-15, 20, 0), 5, 0.5));
        linearSystem.put("Third", new OrientationTerminator(position, new Vector3(-15, 80, 0), 5, 0.5));
        linearSystem.put("Fourth", new OrientationTerminator(position, new Vector3(-10, 70, 0), 5, 0.5));
        linearSystem.put("End", Terminator.nullTerminator());

        stateMachine.appendLogicState("Main", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.update(sensorData, hardwareData);
            }
        });

        stateMachine.activateLogic("Main");
    }
}
