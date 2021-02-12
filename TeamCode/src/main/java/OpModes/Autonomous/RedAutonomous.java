package OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

import Hardware.Hardware;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.RobotSystems.MecanumSystem;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathUtils.Vector2;
import MathUtils.Vector3;
import MathUtils.Vector4;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.PurePursuit.PurePursuitBuilder;
import Motion.Terminators.LogicStateTerminator;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.TimeTerminator;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.DriveStateActivator;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;

@Autonomous
public class RedAutonomous extends BasicOpmode {
    Vector3 position, velocity;
    Vector3 wobble1pos;
    Odometer odometer;
    int stackHeight = 4;
    public RedAutonomous() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        hardware.deRegisterDevice(Hardware.HardwareDevices.WOBBLE);
        hardware.disableDevice(Hardware.HardwareDevices.WOBBLE);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        wobble1pos = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Odometer", odometer);

        eventSystem.onInit("Setup", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardwareData.setShooterTilt(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_HOLD);
                if(isStarted()){
                    deactivateThis();
                }
                telemetry.addData("Init", "true");
            }
        });

        opmodeVariables.integers.put("count", 0);

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("Drive To Left Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(-3.5, 60))
                .setSpeed(0.9)
                .setR1(40)
                .setR2(0.25)
                .setSlowMod(55)
                .setMinimums(0.2)
                .complete());
        driveStates.put("Drive To Centre Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(4.5, 60))
                .setSpeed(0.3)
                .setR1(2)
                .setMinimums(0.2)
                .complete());
        driveStates.put("Drive to Right Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(12.5, 60))
                .setSpeed(0.3)
                .setR1(2)
                .setMinimums(0.2)
                .complete());

        driveStates.put("Drive To Wobble 2", new PurePursuitBuilder(stateMachine, position)
                .addTarget(new Vector2(40, 65))
                .addTarget(new Vector2(45, 65))
                .addTarget(new Vector2(45, 40))
                .setSpeed(0.75)
                .setAngle(-5)
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

        driveStates.put("Stop2", new DriveState(stateMachine) {
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
        autoStates.put("Right Powershot", new DriveStateActivator(stateMachine, "Drive to Right Powershot"));
        autoStates.put("Drive To Wobble 1 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 1"));
        autoStates.put("Release Wobble 1", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooterTilt(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
                telemetry.addData("Dropping", "The wobble Goal");
            }
        });
        autoStates.put("Collect Wobble 2", new DriveStateActivator(stateMachine, "Drive To Wobble 2"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop2"));
        stateMachine.appendLogicStates(autoStates);

        final LinearEventSystem linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);
        linearSystem.put("Left Powershot", new OrientationTerminator(position, new Vector3(-3.5, 60, 0), 0.26, 0.2));
        linearSystem.put("End", new TimeTerminator(100));
        linearSystem.put("Centre Powershot", new OrientationTerminator(position, new Vector3(4.5, 60, 0), 0.26, 0.15));
        linearSystem.put("End", new TimeTerminator(100));
        linearSystem.put("Right Powershot", new OrientationTerminator(position, new Vector3(12.5, 60, 0), 0.26, 0.15));
        linearSystem.put("End", new TimeTerminator(100));
        linearSystem.put("Drive To Wobble 1 Activator", new OrientationTerminator(position, wobble1pos, 0.75, 3));
        linearSystem.put("Release Wobble 1", new TimeTerminator(300));
        linearSystem.put("Collect Wobble 2", new OrientationTerminator(position, new Vector3(45, 40, -5), 1, 1));
        linearSystem.put("End", new TimeTerminator(3000));

        eventSystem.onStart("Jog", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setDriveMotors(MecanumSystem.translate(new Vector3(0, -0.8, 0)));
                if(stackHeight == 4){
                    wobble1pos.set(new Vector3(40, 124, 0));
                }
                stateMachine.appendDriveState("Drive To Wobble 1", new DriveToPointBuilder(stateMachine, position)
                        .setTarget(wobble1pos.getVector2())
                        .setSpeed(0.9)
                        .setR1(20)
                        .setR2(1)
                        .setMinimums(0.25)
                        .setRotPrec(2.5)
                        .complete());
            }
        });

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
