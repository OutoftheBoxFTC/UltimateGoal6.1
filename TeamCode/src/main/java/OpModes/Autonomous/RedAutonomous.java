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
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.PIDDriveToPoint.PIDDriveToPointBuilder;
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
    int stackHeight = 1;
    public RedAutonomous() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
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
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_HOLD);
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.HOLD_INTAKE);
                if(isStarted()){
                    hardware.disableDevice(Hardware.HardwareDevices.WEBCAM);
                    deactivateThis();
                }
                telemetry.addData("Init", "true");
                //telemetry.addData("Tfod", sensorData.getRings());
                //stackHeight = (int)sensorData.getRings();
            }
        });

        opmodeVariables.integers.put("count", 0);

        HashMap<String, DriveState> driveStates = new HashMap<>();
        driveStates.put("Drive To Left Powershot", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(-4.5, 60)) //b should be 60
                .setSpeed(0.7)
                .setRot(-1)
                .setSlowdownDistance(5)
                .setRotationSlowdown(0.1)
                .complete());
        driveStates.put("Drive To Centre Powershot", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(2.5, 60))
                .setSpeed(0.3)
                .setRot(0)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.11, 0, 0))
                .setRotGain(new Vector4(4, 0.5, 0, 0))
                .setMinimums(0.15)
                .complete());
        driveStates.put("Drive to Right Powershot", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(7, 60))
                .setSpeed(0.3)
                .setRot(-1)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.1, 0.1, 0, 0))
                .setRotGain(new Vector4(4, 0.75, 0, 0))
                .setMinimums(0.15)
                .complete());

        driveStates.put("Drive To Wobble 2", new PurePursuitBuilder(stateMachine, position)
                .addTarget(new Vector2(40, 65))
                .addTarget(new Vector2(42, 65))
                .addTarget(new Vector2(42, 45))
                .setSpeed(0.75)
                .setRadius(15)
                .setAngle(0)
                .complete()
        );

        driveStates.put("Collect Wobble 2", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(40, 20))
                .setSpeed(0.1)
                .setRot(-6.5)
                .setSlowdownDistance(2)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.1, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );

        driveStates.put("Drive To Ring Stack", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(30, 20))
                .setSpeed(0.2)
                .setRot(5)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete());

        driveStates.put("Intake Stack 1", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(30, 32.5))
                .setSpeed(0.1)
                .setRot(0)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );

        driveStates.put("Stop", new DriveState(stateMachine) {
            @Override
            public Vector4 getDriveVelocities() {
                return Vector4.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Done", "done");
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

        //Turn Drive States into Logic States - this is for some compatibility stuff
        HashMap<String, LogicState> autoStates = new HashMap<>();
        autoStates.put("Left Powershot", new DriveStateActivator(stateMachine, "Drive To Left Powershot"));
        autoStates.put("Centre Powershot", new DriveStateActivator(stateMachine, "Drive To Centre Powershot"));
        autoStates.put("Right Powershot", new DriveStateActivator(stateMachine, "Drive to Right Powershot"));
        autoStates.put("Drive To Wobble 1 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 1"));
        autoStates.put("Release Wobble 1", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
                telemetry.addData("Dropping", "The wobble Goal");
            }
        });
        autoStates.put("Drive Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 2"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop2"));
        autoStates.put("Shoot", new SingleLogicState(stateMachine) {

            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.activateLogic("ShootMain");
                telemetry.addData("Shooting", true);
            }
        });
        autoStates.put("ShootMain", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 100;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 100;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        state = 0;
                        deactivateThis();
                    }
                }
                telemetry.addData("state", state);
            }
        });
        autoStates.put("Release Forks", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.49055);
                hardwareData.setWobbleLiftLeft(0.4801);
            }
        });
        autoStates.put("Move Forks Down", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.43622);
                hardwareData.setWobbleLiftLeft(0.5208);
            }
        });
        autoStates.put("Raise Forks", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.57676);
                hardwareData.setWobbleLiftLeft(0.37882);
            }
        });
        autoStates.put("Drop And Outtake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                hardwareData.setShooterTilt(0.3373);
                stateMachine.activateLogic("Intake");
            }
        });
        autoStates.put("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Collect Wobble 2 Activator", new DriveStateActivator(stateMachine, "Collect Wobble 2"));
        autoStates.put("Drive To Ring Stack Activator", new DriveStateActivator(stateMachine, "Drive To Ring Stack"));
        autoStates.put("Intake Stack 1 Activator", new DriveStateActivator(stateMachine, "Intake Stack 1"));
        stateMachine.appendLogicStates(autoStates);

        //Auto Linear System in order of states executed
        final LinearEventSystem linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);
        linearSystem.put("Left Powershot", new OrientationTerminator(position, new Vector3(-4.5, 60, -1), 0.5, 0.5));
        linearSystem.put("Shoot", new TimeTerminator(50));
        linearSystem.put("Centre Powershot", new OrientationTerminator(position, new Vector3(2.5, 60, 0), 0.5, 0.75));
        linearSystem.put("Shoot", new TimeTerminator(50));
        linearSystem.put("Right Powershot", new OrientationTerminator(position, new Vector3(7, 60, -1), 0.5, 0.5));
        linearSystem.put("Shoot", new TimeTerminator(50));
        linearSystem.put("Drive To Wobble 1 Activator", new OrientationTerminator(position, wobble1pos, 1.5, 3));
        linearSystem.put("Release Wobble 1", new TimeTerminator(300));
        linearSystem.put("Release Forks", new TimeTerminator(50));
        linearSystem.put("Drive Wobble 2 Activator", new OrientationTerminator(position, new Vector3(42, 45, 0), 3, 1));
        linearSystem.put("Move Forks Down", new TimeTerminator(5));
        linearSystem.put("Collect Wobble 2 Activator", new OrientationTerminator(position, new Vector3(40, 20, -6.5), 4, 2));
        linearSystem.put("Raise Forks", new TimeTerminator(50));
        linearSystem.put("Drive To Ring Stack Activator", new OrientationTerminator(position, new Vector3(30, 20, 5), 4, 2));
        linearSystem.put("Drop And Outtake", new TimeTerminator(150));
        linearSystem.put("Intake Stack 1 Activator", new OrientationTerminator(position, new Vector3(30, 32.5, 0), 4, 1));

        linearSystem.put("End", new TimeTerminator(3000));

        eventSystem.onStart("Jog", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                odometer.reset();
                hardwareData.setDriveMotors(MecanumSystem.translate(new Vector3(0, -0.8, 0)));
                hardwareData.setShooter(0.8);
                hardwareData.setShooterTilt(0.36);
                hardwareData.setShooterLoadArm(0.875);
                if(stackHeight == 4){
                    wobble1pos.set(new Vector3(35, 122, 0));
                }else if(stackHeight == 1){
                    wobble1pos.set(new Vector3(7, 105, 0));
                }
                stateMachine.appendDriveState("Drive To Wobble 1", new DriveToPointBuilder(stateMachine, position)
                        .setTarget(wobble1pos.getVector2())
                        .setSpeed(0.9)
                        .setR1(15)
                        .setR2(1)
                        .setRot(0)
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

        eventSystem.onStart("SpinShooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooter(0.8);
                hardwareData.setShooterTilt(0.36);
            }
        });

        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position.getVector2().toVector3(Math.toDegrees(position.getC())));
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
                telemetry.addData("State", stateMachine.getActiveStates());
            }
        });
    }
}
