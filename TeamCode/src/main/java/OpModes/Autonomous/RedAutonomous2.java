package OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDeviceCloseOnTearDown;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;
import java.util.HashMap;

import Hardware.CustomClasses.SingletonVariables;
import Hardware.Hardware;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.RobotSystems.MecanumSystem;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.SmartDevices.SmartTensorflow.SmartTensorflow;
import Hardware.UltimateGoalHardware;
import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.PIDSystem;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.PIDDriveToPoint.PIDDriveToPointBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Motion.PurePursuit.PurePursuitBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.TimeTerminator;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.DriveStateActivator;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@Autonomous
public class RedAutonomous2 extends BasicOpmode {
    Vector3 position, velocity;
    Vector3 wobble1pos, wobble2pos;
    Odometer odometer;
    int stackHeight = 4;
    double[] powershots = new double[]{0, 0, 0};
    public static double powershot1 = 2.5, powershot2 = -2.5, powershot3 = -7.5;
    public RedAutonomous2() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        wobble1pos = Vector3.ZERO();
        wobble2pos = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Odometer", odometer);

        eventSystem.onInit("Setup", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Calibration Pitch", hardware.getSmartDevices().get("SmartCV", SmartCV.class).calibratePitch());
                telemetry.addData("Calibration Range", sensorData.getRange());
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_HOLD);
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.HOLD_INTAKE);
                hardwareData.setShooterLoadArm(0.875);
                if(isStarted()){
                    hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                    deactivateThis();
                }
                telemetry.addData("Init", "true");
                telemetry.addData("Tfod", sensorData.getRings());
                stackHeight = (int)sensorData.getRings();
            }
        });

        //Auto Linear System in order of states executed
        final LinearEventSystem linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);

        eventSystem.onStart("Jog", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                odometer.reset();
                hardwareData.setDriveMotors(MecanumSystem.translate(new Vector3(0, -0.8, 0)));
                hardwareData.setShooter(0.8);
                hardwareData.setShooterTilt(0.36);
                hardwareData.setShooterLoadArm(0.925);

                Path path = new PathBuilder(0, 0, Angle.degrees(0))
                        .lineTo(-7, 50)
                        .complete();
                CrosstrackBuilder builder = new CrosstrackBuilder(stateMachine, position);
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.IDLE_INTAKE);
                stateMachine.appendDriveState("Drive To Powershots", builder.follow(path, 0, 0.5, 0.15));
                stateMachine.appendLogicState("Activate Powershots", new DriveStateActivator(stateMachine, "Drive To Powershots"));
                stateMachine.appendLogicState("Zero Out", new DriveStateActivator(stateMachine, "Rotate To Zero"));

                //stateMachine.appendDriveState("Drive To Left Powershot", builder.follow(powershotPath, 0, 0.55, 0.15));
                linearSystem.put("Activate Powershots", new OrientationTerminator(position, path.getEndpoint().getVector2().toVector3(0), 2, 2));
                linearSystem.put("Zero Out", new OrientationTerminator(position, path.getEndpoint().getVector2().toVector3(-1.5), 10, 1, 5));
                linearSystem.put("End", new TrueTimeTerminator(50));
                linearSystem.put("Get Pos", new TimeTerminator(3));
                linearSystem.put("Shoot First", new TrueTimeTerminator(500));
                linearSystem.put("Load Shooter", new TrueTimeTerminator(550));
                linearSystem.put("Shoot Second", new TrueTimeTerminator(500));
                linearSystem.put("Load Shooter", new TrueTimeTerminator(400));
                linearSystem.put("Shoot Third", new TrueTimeTerminator(500));
                linearSystem.put("Load Shooter", new TrueTimeTerminator(400));

                if(stackHeight == 0){
                    setStackHeight0(linearSystem);
                }else if(stackHeight == 1){
                    setStackHeight1(linearSystem);
                }else{
                    setStackHeight4(linearSystem);
                }
                stateMachine.activateLogic("Main");
            }
        });

        stateMachine.appendDriveState("Rotate To Zero", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                return new Vector3(0, 0, 0.15 * MathUtils.sign(MathUtils.getRadRotDist(position.getC(), Math.toRadians(-1.5))));
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        eventSystem.onStart("Load Shooter", new LogicState(stateMachine) {
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
                    hardwareData.setShooterLoadArm(0.925);
                    timer = System.currentTimeMillis() + 120;
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

        stateMachine.appendDriveState("Drive To Left Powershot", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, 55)) //b should be 60
                .setSpeed(0.6)
                .setRot(0)
                .setRotPrec(1.25)
                .setR2(0.75)
                .setR1(30)
                .setSlowMod(35)
                .setMinimums(0.15)
                .complete()); //Rotate To Left Powershot

        eventSystem.onInit("Get Pos", new LogicState(stateMachine) {
            double powershotDist = -8;
            Vector2 psht1 = new Vector2(-8, 142);
            Vector2 psht2 = new Vector2(psht1.getA()+powershotDist, 142);
            Vector2 psht3 = new Vector2(psht2.getA()+powershotDist, 142);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //powershots = hardware.getSmartDevices().get("SmartCV", SmartCV.class).getPowershots();
                double ang1 = -Math.atan2(psht1.getA() - position.getA(), psht1.getB() - position.getB());
                double ang2 = -Math.atan2(psht2.getA() - position.getA(), psht2.getB() - position.getB());
                double ang3 = -Math.atan2(psht3.getA() - position.getA(), psht3.getB() - position.getB());
                //powershots = new double[]{Math.toDegrees(MathUtils.getRadRotDist(ang1, position.getC())), Math.toDegrees(MathUtils.getRadRotDist(ang2, position.getC())), Math.toDegrees(MathUtils.getRadRotDist(ang3, position.getC()))};
                powershots = new double[]{powershot1, powershot2, powershot3};
            }
        });

        eventSystem.onInit("Shoot First", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {

            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[0]));
                hardwareData.setShooterTilt(0.34);
            }
        });

        stateMachine.appendLogicState("Shoot Second", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[1]));
                hardwareData.setShooterTilt(0.35);
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.HOLD_INTAKE);
            }
        });

        stateMachine.appendLogicState("Shoot Third", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[2]));
                hardwareData.setShooterTilt(0.34);
            }
        });

        stateMachine.appendDriveState("Shoot Turn", new VelocityDriveState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public Vector3 getVelocities() {
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //hardwareData.setShooterLoadArm(0.7);
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 150;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.925);
                    timer = System.currentTimeMillis() + 150;
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

        stateMachine.appendLogicState("Main", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.update(sensorData, hardwareData);
            }
        });

        eventSystem.onInit("Report", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                Telemetry t = FtcDashboard.getInstance().getTelemetry();
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                t.addData("Shooter", vel);
                t.update();
            }
        });

        eventSystem.onStart("SpinShooter", new LogicState(stateMachine) {
            PIDSystem system = new PIDSystem(0.8, 0, 0);
            double target = 3.75, tilt = 0.34;
            boolean lastLeft = false, lastRight = false, lastdown = false, lastup = false;

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                hardwareData.setShooter(0.7 + system.getCorrection(target - vel)); //0.75 | 4.5
                //hardwareData.setShooter(1);
                telemetry.addData("Vel", target);
                if(stateMachine.logicStateActive("SpinShooter2")){
                    deactivateThis();
                }
            }
        });

        stateMachine.appendLogicState("SpinShooter2", new LogicState(stateMachine) {
            PIDSystem system = new PIDSystem(0.8, 0, 0);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                hardwareData.setShooter(0.75 + system.getCorrection(4 - vel));
                hardwareData.setShooterTilt(0.335);
            }
        });

        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position.getVector2().toVector3(Math.toDegrees(position.getC())));
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
                telemetry.addData("State", stateMachine.getActiveStates());
                DecimalFormat format = new DecimalFormat("#.##");

                telemetry.addData("Powershots", format.format(powershots[0]) + " " + format.format(powershots[1]) + " " + format.format(powershots[2]));
                SingletonVariables.getInstance().setPosition(position);
            }
        });
    }

    public void setStackHeight1(LinearEventSystem linearSystem){
        wobble1pos.set(new Vector3(7, 100, 0));
        wobble2pos.set(new Vector3(12, 83, 0));

        CrosstrackBuilder builder = new CrosstrackBuilder(stateMachine, position);
        Path wobble1Path = new PathBuilder(0, 55, Angle.degrees(14)).lineTo(wobble1pos.getVector2(), Angle.degrees(0)).complete();
        Path bouncebackPath = new PathBuilder(wobble1pos)
                .lineTo(-16, 114, Angle.degrees(45))
                .lineTo(-30, 114, Angle.degrees(90))
                .complete();
        Path wobble2Path = new PathBuilder(34, 65, Angle.degrees(0))
                .lineTo(35, 48)
                .complete();
        Path dump2Path = new PathBuilder(19, 38, Angle.degrees(4))
                .lineTo(wobble2pos.getVector2(), Angle.degrees(180))
                .complete();
        Path collect2Path = new PathBuilder(wobble2Path.getEndpoint())
                .lineTo(35, 27, Angle.degrees(0))
                .complete();

        linearSystem.put("Drop And Outtake", new TimeTerminator(10));
        linearSystem.put("Drive To Wobble 1 Activator", new OrientationTerminator(position, wobble1pos, 5, 3));
        linearSystem.put("Release Wobble 1", new TimeTerminator(15));
        //linearSystem.put("Bounceback", new OrientationTerminator(position, bouncebackPath.getEndpoint().getVector2().toVector3(90), 5, 3));
        linearSystem.put("Clear Wobble 1 Activator", new OrientationTerminator(position, new Vector3(7, 65, 0), 5, 10));
        linearSystem.put("Release Forks", new TimeTerminator(10));
        //linearSystem.put("Clear Ring Stack Activator", new OrientationTerminator(position, new Vector3(32, 65, 0), 2.5, 10));
        linearSystem.put("Drive Wobble 2 Activator", new OrientationTerminator(position, new Vector3(34, 45, 0), 5, 1));
        linearSystem.put("Move Forks Down", new TimeTerminator(5));
        linearSystem.put("Collect Wobble 2 Activator", new OrientationTerminator(position, new Vector3(35, 27, 0), 3, 1));
        linearSystem.put("Raise Forks", new TimeTerminator(50));
        linearSystem.put("Drive To Ring Stack Activator", new OrientationTerminator(position, new Vector3(18, 20, 4), 2, 2));
        linearSystem.put("Intake Stack 1 Activator", new OrientationTerminator(position, new Vector3(19, 38, 4.0000001), 2, 1));
        linearSystem.put("End", new TimeTerminator(10));
        linearSystem.put("Flick Shooter", new TrueTimeTerminator(3000));
        linearSystem.put("Auto Aim", new TimeTerminator(2));
        linearSystem.put("Repeat Shoot", new TimeTerminator(2));
        linearSystem.put("Dump Wobble 2 Activator", new OrientationTerminator(position, wobble2pos.getVector2().toVector3(180), 5, 5));
        linearSystem.put("Rotate 180 Activator", new OrientationTerminator(position, new Vector3(0, 0, 190), 500, 10));
        linearSystem.put("End", new TimeTerminator(10));
        linearSystem.put("Move Forks Down", new TimeTerminator(10));
        linearSystem.put("Park Activator", new TimeTerminator(1000));
        linearSystem.put("End", new TimeTerminator(3000));

        HashMap<String, DriveState> driveStates = new HashMap<>();
        /**
        driveStates.put("Collect Wobble 2", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(35, 27))
                .setSpeed(0.1)
                .setSlowdownDistance(2)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.1, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );
         */
        driveStates.put("Collect Wobble 2", builder.follow(collect2Path, 0, 0.2, 0.15));

        driveStates.put("Intake Stack 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(19, 38))
                .setSpeed(0.5)
                .setRot(4)
                .setRotPrec(1)
                .complete()
        );

        driveStates.put("Rotate 180", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, 0))
                .setR2(500)
                .setRot(190)
                .setSpeed(0.75)
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
        /**
        stateMachine.appendDriveState("Drive To Wobble 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(wobble1pos.getVector2())
                .setSpeed(0.9)
                .setR1(10)
                .setR2(1)
                .setRot(0)
                .setMinimums(0.15)
                .setRotPrec(2.5)
                .complete());
         */
        stateMachine.appendDriveState("Drive To Wobble 1", builder.follow(wobble1Path, 0, 0.9, 0.15));
        stateMachine.appendDriveState("Grab Bouncebacks", builder.follow(bouncebackPath, 0, 1, 0.05));
        /**
        stateMachine.appendDriveState("Drive To Dump Wobble 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(wobble2pos.getVector2())
                .setSpeed(0.5)
                .setR1(30)
                .setMinimums(0.2)
                .setRotPrec(5)
                .setRot(0)
                .complete());
         */
        stateMachine.appendDriveState("Drive To Dump Wobble 2", builder.follow(dump2Path, 0, 0.5, 0.15));
        stateMachine.appendDriveState("Drive To Clear Wobble 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(7, 65))
                .setSpeed(0.75)
                .complete());
        stateMachine.appendDriveState("Drive To Clear Ring Stack", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(32, 65))
                .setSpeed(0.5)
                .complete());
        /**
        stateMachine.appendDriveState("Drive To Wobble 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(32, 45))
                .setSpeed(0.6)
                .setR1(20)
                .setMinimums(0.15)
                .complete());
         */
        stateMachine.appendDriveState("Drive To Wobble 2", builder.follow(wobble2Path, 0, 0.8, 0.15));

        stateMachine.appendDriveState("Drive To Ring Stack", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(20, 20))
                .setSpeed(0.3)
                .setRot(5)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete());
        stateMachine.appendDriveState("Park", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(wobble2pos.getA(), 67))
                .setSpeed(0.6)
                .setR2(5)
                .setRotPrec(50)
                .setRot(170)
                .complete());

        stateMachine.appendDriveState("Drive To Ring Bounce", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(11, 115))
                .setSpeed(0.7)
                .setR1(30)
                .setR2(1)
                .setRot(0)
                .setMinimums(0.15)
                .setRotPrec(2.5)
                .complete());

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
                hardwareData.setIntakePower(0);
                telemetry.addData("Dropping", "The wobble Goal");
            }
        });
        autoStates.put("Bounceback", new DriveStateActivator(stateMachine, "Grab Bouncebacks"));
        autoStates.put("Clear Wobble 1 Activator", new DriveStateActivator(stateMachine, "Drive To Clear Wobble 1"));
        autoStates.put("Clear Ring Stack Activator", new DriveStateActivator(stateMachine, "Drive To Clear Ring Stack"));
        autoStates.put("Drive Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 2"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop2"));
        autoStates.put("Shoot", new DriveStateActivator(stateMachine, "Shoot Turn"));
        autoStates.put("Dump Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Dump Wobble 2"));
        autoStates.put("Rotate 180 Activator", new DriveStateActivator(stateMachine, "Rotate 180"));
        autoStates.put("Park Activator", new DriveStateActivator(stateMachine, "Park"));
        autoStates.put("Flick Shooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.activateLogic("Auto Aim");
            }
        });
        autoStates.put("Auto Aim", new LogicState(stateMachine) {
            long timer = 0;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                timer = System.currentTimeMillis() + 250;
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack()){
                    double angDelta = Math.toRadians(hardware.smartDevices.get("SmartCV", SmartCV.class).getHeading());
                    hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));
                }
                if(System.currentTimeMillis() > timer){
                    stateMachine.activateLogic("Repeat Shoot");
                }
            }
        });
        autoStates.put("Repeat Shoot", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //hardwareData.setShooterLoadArm(0.7);
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 250;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.925);
                    timer = System.currentTimeMillis() + 250;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        state = 0;
                    }
                }
                telemetry.addData("state", state);
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
                    hardwareData.setShooterLoadArm(0.925);
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
                hardwareData.setShooterTilt(0.34);
            }
        });
        autoStates.put("Move Forks Down", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.43622);
                hardwareData.setWobbleLiftLeft(0.5208);
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                hardwareData.setShooterTilt(0.34);
            }
        });
        autoStates.put("Raise Forks", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.57676);
                hardwareData.setWobbleLiftLeft(0.37882);
                hardwareData.setShooterLoadArm(0.875);
                hardwareData.setShooterTilt(0.34);
            }
        });
        autoStates.put("Drop And Outtake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                stateMachine.activateLogic("Intake");
                stateMachine.activateLogic("SpinShooter2");
            }
        });
        autoStates.put("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Stop Shooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.deactivateState("SpinShooter");
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                //stateMachine.activateLogic("Intake");
                //hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Collect Wobble 2 Activator", new DriveStateActivator(stateMachine, "Collect Wobble 2"));
        autoStates.put("Drive To Ring Stack Activator", new DriveStateActivator(stateMachine, "Drive To Ring Stack"));
        autoStates.put("Intake Stack 1 Activator", new DriveStateActivator(stateMachine, "Intake Stack 1"));
        autoStates.put("Drive To Ring Bounce Activator", new DriveStateActivator(stateMachine, "Drive To Ring Bounce"));
        stateMachine.appendLogicStates(autoStates);
    }

    public void setStackHeight0(LinearEventSystem linearSystem){
        wobble1pos.set(new Vector3(35, 86, 0));
        wobble2pos.set(new Vector3(22, 79, 0));
        CrosstrackBuilder builder = new CrosstrackBuilder(stateMachine, position);
        Path parkPath = new PathBuilder(wobble2pos.getVector2(), Angle.degrees(90))
                .lineTo(10, 81, Angle.degrees(90))
                .lineTo(0, 81, Angle.degrees(0))
                .complete();

        linearSystem.put("Drive To Wobble 1 Activator", new OrientationTerminator(position, wobble1pos, 5, 3));
        linearSystem.put("Release Wobble 1", new TimeTerminator(50));
        linearSystem.put("Release Forks", new TimeTerminator(50));
        linearSystem.put("Drive Wobble 2 Activator", new OrientationTerminator(position, new Vector3(36, 45, 0), 3, 1));
        linearSystem.put("Move Forks Down", new TimeTerminator(5));
        linearSystem.put("Collect Wobble 2 Activator", new OrientationTerminator(position, new Vector3(35, 25, 0), 3, 1));
        linearSystem.put("Raise Forks", new TimeTerminator(50));
        //linearSystem.put("Drive To Ring Stack Activator", new OrientationTerminator(position, new Vector3(20, 20, 5), 1, 2));
        //linearSystem.put("Drop And Outtake", new TimeTerminator(150));
        //linearSystem.put("Intake Stack 1 Activator", new OrientationTerminator(position, new Vector3(20, 32.5, -5), 6, 2));
        //linearSystem.put("End", new TimeTerminator(200));
        //linearSystem.put("Shoot", new TimeTerminator(40));
        linearSystem.put("Dump Wobble 2 Activator", new OrientationTerminator(position, wobble2pos, 5, 5));
        linearSystem.put("Rotate 90 Activator", new OrientationTerminator(position, new Vector3(20, 0, 90), 500, 10));
        linearSystem.put("Move Forks Down", new TimeTerminator(50));
        linearSystem.put("Park Activator", new OrientationTerminator(position, parkPath.getEndpoint(), 5, 3));
        //linearSystem.put("Grab Bounceback Activator", new OrientationTerminator(position, new Vector3(20, 120, 0), 2, 2));
        linearSystem.put("End", new TimeTerminator(3000));

        HashMap<String, DriveState> driveStates = new HashMap<>();

        driveStates.put("Collect Wobble 2", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(35, 25))
                .setSpeed(0.1)
                .setSlowdownDistance(2)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.1, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );

        driveStates.put("Intake Stack 1", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(20, 32.5))
                .setSpeed(0.3)
                .setRot(-5)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );

        driveStates.put("Rotate 90", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, 0))
                .setR2(500)
                .setRot(90)
                .setSpeed(0.5)
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

        stateMachine.appendDriveState("Drive To Wobble 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(wobble1pos.getVector2())
                .setSpeed(0.9)
                .setR1(30)
                .setR2(1)
                .setRot(0)
                .setMinimums(0.15)
                .setRotPrec(2.5)
                .complete());
        stateMachine.appendDriveState("Drive To Dump Wobble 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(wobble2pos.getVector2())
                .setSpeed(0.5)
                .setR1(30)
                .setMinimums(0.2)
                .setRotPrec(5)
                .setRot(0)
                .complete());
        stateMachine.appendDriveState("Drive To Wobble 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(36, 45))
                .setSpeed(0.5)
                .setR1(20)
                .setMinimums(0.2)
                .setRotPrec(2)
                .complete());
        stateMachine.appendDriveState("Drive To Ring Stack", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(20, 20))
                .setSpeed(0.2)
                .setRot(5)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete());

        /**
        stateMachine.appendDriveState("Park", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, wobble2pos.getB()))
                .setSpeed(0.3)
                .setR2(5)
                .setRotPrec(50)
                .setRot(90)
                .complete());
        */
        stateMachine.appendDriveState("Park", builder.follow(parkPath, 0, 0.3, 0.1));
        stateMachine.appendDriveState("Grab Bounceback", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(20, 90))
                .setSpeed(0.75)
                .setRot(0)
                .setMinimums(0.2)
                .setRotPrec(2)
                .complete());
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
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                telemetry.addData("Dropping", "The wobble Goal");
            }
        });
        autoStates.put("Drive Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 2"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop2"));
        autoStates.put("Shoot", new DriveStateActivator(stateMachine, "Shoot Turn"));
        autoStates.put("Dump Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Dump Wobble 2"));
        autoStates.put("Rotate 90 Activator", new DriveStateActivator(stateMachine, "Rotate 90"));
        autoStates.put("Park Activator", new DriveStateActivator(stateMachine, "Park"));
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
                    hardwareData.setShooterLoadArm(0.865);
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
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Raise Forks", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.57676);
                hardwareData.setWobbleLiftLeft(0.37882);
                hardwareData.setShooterLoadArm(0.875);
            }
        });
        autoStates.put("Drop And Outtake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                stateMachine.activateLogic("Intake");
                stateMachine.activateLogic("SpinShooter2");
            }
        });
        autoStates.put("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Stop Shooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.deactivateState("SpinShooter");
            }
        });
        autoStates.put("Collect Wobble 2 Activator", new DriveStateActivator(stateMachine, "Collect Wobble 2"));
        autoStates.put("Drive To Ring Stack Activator", new DriveStateActivator(stateMachine, "Drive To Ring Stack"));
        autoStates.put("Intake Stack 1 Activator", new DriveStateActivator(stateMachine, "Intake Stack 1"));
        autoStates.put("Grab Bounceback Activator", new DriveStateActivator(stateMachine, "Grab Bounceback"));
        stateMachine.appendLogicStates(autoStates);
    }

    public void setStackHeight4(LinearEventSystem linearSystem){
        wobble1pos.set(new Vector3(30, 120, 0));
        wobble2pos.set(new Vector3(28, 112, 0));

        CrosstrackBuilder builder = new CrosstrackBuilder(stateMachine, position);
        Path wobble1Path = new PathBuilder(0, 55, Angle.degrees(14)).lineTo(wobble1pos.getVector2(), Angle.degrees(45)).complete();
        Path bouncebackPath = new PathBuilder(wobble1Path.getEndpoint())
                .lineTo(-16, 114, Angle.degrees(45))
                .lineTo(-32, 114, Angle.degrees(90))
                .complete();
        Path wobble2Path = new PathBuilder(bouncebackPath.getEndpoint())
                .lineTo(27, 60, Angle.degrees(41))
                .lineTo(32, 45, Angle.degrees(0)).complete();
        Path collectWobble2Path = new PathBuilder(wobble2Path.getEndpoint())
                .lineTo(32, 22, Angle.degrees(0))
                .complete();
        Path dumpWobble2Path = new PathBuilder(22, 58, Angle.degrees(6))
                .lineTo(wobble2pos.getA(), wobble2pos.getB() - 15, Angle.degrees(0))
                .lineTo(wobble2pos.getVector2(), Angle.degrees(90))
                .complete();

        linearSystem.put("Drop And Outtake", new TimeTerminator(5));
        linearSystem.put("Drive To Wobble 1 Activator", new OrientationTerminator(position, wobble1pos.getVector2().toVector3(45), 5, 3));
        linearSystem.put("Release Wobble 1", new TimeTerminator(2));
        linearSystem.put("Drive To Bounceback", new OrientationTerminator(position, bouncebackPath.getEndpoint().getVector2().toVector3(90), 5, 3));
        linearSystem.put("Drive Wobble 2 Activator", new OrientationTerminator(position, new Vector3(34, 45, 0), 5, 1));
        linearSystem.put("Release Forks", new TimeTerminator(10));
        linearSystem.put("Move Forks Down", new TimeTerminator(2));
        linearSystem.put("Collect Wobble 2 Activator", new OrientationTerminator(position, collectWobble2Path.getEndpoint(), 5, 1));
        linearSystem.put("Raise Forks", new TimeTerminator(2));
        linearSystem.put("Drive To Ring Stack Activator", new OrientationTerminator(position, new Vector3(22, 18, 5), 3, 2.5));
        linearSystem.put("Intake Stack 1 Activator", new OrientationTerminator(position, new Vector3(22, 31, 6), 5, 2));
        linearSystem.put("End", new TimeTerminator(30));
        linearSystem.put("Flick Shooter", new TrueTimeTerminator(750));
        //linearSystem.put("Shoot", new TimeTerminator(30));
        //linearSystem.put("ShootMain", new TimeTerminator(20));
        linearSystem.put("Back Up Activator", new TimeTerminator(10));
        linearSystem.put("Intake Stack 2 Activator", new OrientationTerminator(position, new Vector3(22, 56, 6), 3, 2));
        linearSystem.put("End", new TimeTerminator(2));
        linearSystem.put("Flick Shooter", new TrueTimeTerminator(1000));
        //linearSystem.put("Shoot", new TimeTerminator(30));
        //linearSystem.put("ShootMain", new TimeTerminator(30));
        //linearSystem.put("ShootMain", new TimeTerminator(30));
        linearSystem.put("Auto Aim", new TimeTerminator(2));
        linearSystem.put("Repeat Shoot", new TimeTerminator(2));
        linearSystem.put("Dump Wobble 2 Activator", new OrientationTerminator(position, dumpWobble2Path.getEndpoint().getVector2().toVector3(90), 5, 5));
        linearSystem.put("Rotate 180 Activator", new OrientationTerminator(position, new Vector3(0, 0, 90), 500, 10));
        linearSystem.put("End", new TimeTerminator(5));
        linearSystem.put("Move Forks Down", new TimeTerminator(5));
        linearSystem.put("Stick", new TimeTerminator(2));
        linearSystem.put("Park Activator", new TimeTerminator(1000));
        linearSystem.put("End", new TimeTerminator(3000));

        HashMap<String, DriveState> driveStates = new HashMap<>();
        /**
        driveStates.put("Drive To Left Powershot", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(-6, 60)) //b should be 60
                .setSpeed(0.6)
                .setRot(6)
                .setSlowdownDistance(20)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.02, 0.07, 0, 0))
                .setRotGain(new Vector4(5, 0.1, 0, 0))
                .complete());
         */

        driveStates.put("Bounceback", builder.follow(bouncebackPath, 0, 1, 0.05));


        /**
        driveStates.put("Collect Wobble 2", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(40, 27))
                .setSpeed(0.2)
                .setSlowdownDistance(2)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.1, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete()
        );
         */
        driveStates.put("Collect Wobble 2", builder.follow(collectWobble2Path, 0, 0.25, 0.5));

        driveStates.put("Intake Stack 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(22, 31))
                .setSpeed(0.75)
                .setRot(5)
                .setRotPrec(1)
                .complete()
        );

        driveStates.put("Intake Stack 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(22, 56))
                .setSpeed(0.3)
                .setRot(6)
                .setRotPrec(1)
                .setMinimums(0.25)
                .complete());

        driveStates.put("Back Up", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        return new Vector3(0, 0.4, 0);
                    }

                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {

                    }
                }
        );

        driveStates.put("Rotate 180", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(0, 0))
                .setR2(500)
                .setRot(90)
                .setSpeed(0.75)
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
        /**
        stateMachine.appendDriveState("Drive To Wobble 1", new DriveToPointBuilder(stateMachine, position)
                .setTarget(wobble1pos.getVector2())
                .setSpeed(0.9)
                .setR1(30)
                .setR2(1)
                .setRot(0)
                .setMinimums(0.15)
                .setRotPrec(2.5)
                .complete());
         */
        stateMachine.appendDriveState("Drive To Wobble 1", builder.follow(wobble1Path, 0, 1, 0.1));
        stateMachine.appendDriveState("Drive To Dump Wobble 2", builder.follow(dumpWobble2Path, 0, 1, 0.1));
        /**
        stateMachine.appendDriveState("Drive To Wobble 2", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(43, 45))
                .setSpeed(0.8)
                .setR1(30)
                .setSlowMod(75)
                .setMinimums(0.15)
                .complete());
        */
        stateMachine.appendDriveState("Drive To Wobble 2", builder.follow(wobble2Path, 0, 1, 0.1));

        stateMachine.appendDriveState("Drive To Ring Stack", new PIDDriveToPointBuilder(stateMachine, position, velocity)
                .setTarget(new Vector2(22, 18))
                .setSpeed(0.3)
                .setRot(5)
                .setSlowdownDistance(1)
                .setRotationSlowdown(0.1)
                .setDriveGain(new Vector4(0.15, 0.095, 0, 0))
                .setRotGain(new Vector4(4, 0.3, 0, 0))
                .complete());
        stateMachine.appendDriveState("Park", new DriveToPointBuilder(stateMachine, position)
                .setTarget(new Vector2(wobble2pos.getA(), 75))
                .setSpeed(0.7)
                .setMinimums(0.4)
                .setR2(5)
                .setRotPrec(50)
                .setRot(170)
                .complete());

        stateMachine.appendDriveStates(driveStates);

        //Turn Drive States into Logic States - this is for some compatibility stuff
        HashMap<String, LogicState> autoStates = new HashMap<>();
        autoStates.put("Left Powershot", new DriveStateActivator(stateMachine, "Drive To Left Powershot"));
        autoStates.put("Rot To Left", new DriveStateActivator(stateMachine, "Rotate To Left Powershot"));
        autoStates.put("Centre Powershot", new DriveStateActivator(stateMachine, "Drive To Centre Powershot"));
        autoStates.put("Right Powershot", new DriveStateActivator(stateMachine, "Drive to Right Powershot"));
        autoStates.put("Drive To Bounceback", new DriveStateActivator(stateMachine, "Bounceback"));
        autoStates.put("Drive To Wobble 1 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 1"));
        autoStates.put("Release Wobble 1", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
                hardwareData.setIntakePower(0);
                telemetry.addData("Dropping", "The wobble Goal");
            }
        });
        autoStates.put("Drive Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Wobble 2"));
        autoStates.put("End", new DriveStateActivator(stateMachine, "Stop"));
        autoStates.put("End2", new DriveStateActivator(stateMachine, "Stop2"));
        autoStates.put("Shoot", new DriveStateActivator(stateMachine, "Shoot Turn"));
        autoStates.put("Dump Wobble 2 Activator", new DriveStateActivator(stateMachine, "Drive To Dump Wobble 2"));
        autoStates.put("Rotate 180 Activator", new DriveStateActivator(stateMachine, "Rotate 180"));
        autoStates.put("Park Activator", new DriveStateActivator(stateMachine, "Park"));
        autoStates.put("ShootMain", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //hardwareData.setShooterLoadArm(0.7);
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
                    hardwareData.setShooterLoadArm(0.925);
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
                hardwareData.setShooterTilt(0.34);
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
            }
        });
        autoStates.put("Stick", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
            }
        });
        autoStates.put("Raise Forks", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLiftRight(0.57676);
                hardwareData.setWobbleLiftLeft(0.37882);
                hardwareData.setShooterLoadArm(0.875);
            }
        });
        autoStates.put("Drop And Outtake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                stateMachine.activateLogic("Intake");
                stateMachine.activateLogic("SpinShooter2");
            }
        });
        autoStates.put("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Flick Shooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.activateLogic("Auto Aim");
            }
        });
        autoStates.put("Auto Aim", new LogicState(stateMachine) {
            long timer = 0;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                timer = System.currentTimeMillis() + 250;
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack()){
                    double angDelta = Math.toRadians(hardware.smartDevices.get("SmartCV", SmartCV.class).getHeading());
                    hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));
                }
                if(System.currentTimeMillis() > timer){
                    stateMachine.activateLogic("Repeat Shoot");
                }
            }
        });
        autoStates.put("Repeat Shoot", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //hardwareData.setShooterLoadArm(0.7);
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 250;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.925);
                    timer = System.currentTimeMillis() + 250;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        state = 0;
                    }
                }
                telemetry.addData("state", state);
            }
        });
        autoStates.put("Stop Shooter", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.deactivateState("SpinShooter");
                hardwareData.setIntakePower(1);
            }
        });
        autoStates.put("Collect Wobble 2 Activator", new DriveStateActivator(stateMachine, "Collect Wobble 2"));
        autoStates.put("Drive To Ring Stack Activator", new DriveStateActivator(stateMachine, "Drive To Ring Stack"));
        autoStates.put("Intake Stack 1 Activator", new DriveStateActivator(stateMachine, "Intake Stack 1"));
        autoStates.put("Intake Stack 2 Activator", new DriveStateActivator(stateMachine, "Intake Stack 2"));
        autoStates.put("Back Up Activator", new DriveStateActivator(stateMachine, "Back Up"));
        stateMachine.appendLogicStates(autoStates);
    }
}

