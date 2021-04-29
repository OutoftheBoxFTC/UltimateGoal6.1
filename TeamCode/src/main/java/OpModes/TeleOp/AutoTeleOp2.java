package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;
import java.util.HashMap;

import Hardware.CustomClasses.SingletonVariables;
import Hardware.Hardware;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartCV.TowerCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.ProgramClock;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@TeleOp
@Config
public class AutoTeleOp2 extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean holdShoot = true;
    boolean shot = false;
    boolean shoot1 = false, shoot2 = false, shoot3 = false;
    boolean stopShooter = false;
    int timer = 0;
    long turretTimer = 0;
    double rotOffset;
    public static double p = 1.5, i = 0, d = 0, f = 0.1;
    public AutoTeleOp2() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        //hardware.disableDevice(Hardware.HardwareDevices.WEBCAM);
        //hardware.deRegisterDevice(Hardware.HardwareDevices.WEBCAM);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("odo", odometer);
        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Hit start to start");
                telemetry.addData("Singleton", SingletonVariables.getInstance().getPosition());
                odometer.set(SingletonVariables.getInstance().getPosition());

                hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                hardware.smartDevices.get("SmartCV", SmartCV.class).setPitchOffset(SingletonVariables.getInstance().getPitchOffset());

                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        eventSystem.onStart("GamepadDrive", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                if(gamepad1.right_trigger > 0.1) {
                    double speedMod = (gamepad1.right_trigger != 0) ? 0.8 : 1;
                    hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, -gamepad1.right_stick_x * 0.6);
                }else{
                    if(gamepad1.left_trigger < 0.1) {
                        hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                }
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
            }
        });

        stateMachine.appendDriveState("LinearMove", new VelocityDriveState(stateMachine) {
            long shot1, shot2, shot3;
            @Override
            public Vector3 getVelocities() {
                return new Vector3(0, 0, 0.15);
            }

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                shot1 = System.currentTimeMillis() - 10;
                shot2 = shot1 + 800;
                shot3 = shot2 + 800;
                //stateMachine.activateLogic("Load Shooter Turn");
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooter(1);
                hardwareData.setShooterTilt(0.36);
                if(System.currentTimeMillis() > shot1){
                    //hardwareData.setShooterLoadArm(0.7);
                    stateMachine.activateLogic("Load Shooter Turn");
                    shot1 = Long.MAX_VALUE;
                }
                if(System.currentTimeMillis() > shot2){
                    //hardwareData.setShooterLoadArm(0.7);
                    stateMachine.activateLogic("Load Shooter Turn");
                    shot2 = Long.MAX_VALUE;
                }
                if(System.currentTimeMillis() > shot3){
                    stateMachine.activateLogic("Load Shooter Turn");
                    stateMachine.setActiveDriveState("GamepadDrive");
                    deactivateThis();
                }
            }
        });

        stateMachine.appendDriveState("Rotate", new VelocityDriveState(stateMachine) {
            long timer = 0;
            @Override
            public Vector3 getVelocities() {
                return new Vector3(0, 0, 0.2);
            }

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.y) {
                    timer = System.currentTimeMillis() + 300;
                }else{
                    timer = System.currentTimeMillis() + 200;
                }
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(System.currentTimeMillis() > timer){
                    stateMachine.setActiveDriveState("GamepadDrive");
                }
            }
        });

        stateMachine.appendDriveState("Rotate To Target", new VelocityDriveState(stateMachine) {
            private final Vector2 targetPosition = new Vector2(5, 144);
            double angDelta;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                timer=0;
            }

            @Override
            public Vector3 getVelocities() {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));
                telemetry.addData("Turning", hardwareData.getTurret());
                double deltaX = targetPosition.getA()-position.getA();
                double deltaY = targetPosition.getB()-position.getB();
                //angDelta = MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY)) + Math.toRadians(rotOffset);

                if(sensorData.getTrack()){
                    angDelta = Math.toRadians(sensorData.getHeading());
                }

                double maxSpeed = Math.sqrt(2 * RobotConstants.UltimateGoal.MAX_R_ACCEL * Math.abs(angDelta));
                maxSpeed = maxSpeed/RobotConstants.UltimateGoal.MAX_ROTATION_SPEED;
                maxSpeed = Math.max(maxSpeed, RobotConstants.UltimateGoal.KF);
                shot = timer > 2;

                if(!UGUtils.inRange(Math.toDegrees(angDelta))){
                    timer = 0;
                }else {
                    if(turretTimer < 0 || true) {
                        if(Math.abs(gamepad1.left_stick_x) < 0.1 && Math.abs(gamepad1.left_stick_y) < 0.1 && Math.abs(gamepad1.right_stick_x) < 0.1) {
                            timer += 1;
                        }
                    }
                }
            }
        });

        eventSystem.onStart("Turret", new LogicState(stateMachine) {
            private final Vector2 targetPosition = new Vector2(5, 144);
            double angDelta, prevAng;
            LinearEventSystem system;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                prevAng = 0;
                angDelta = 0;
                system = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.LOOP);
                stateMachine.appendLogicState("Move One", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        shoot1 = true;
                        shoot3 = false;
                    }
                });
                stateMachine.appendLogicState("Move Two", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        shoot2 = true;
                        shoot1 = false;
                    }
                });
                stateMachine.appendLogicState("Move Three", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        shoot3 = true;
                        shoot2 = false;
                    }
                });
                stateMachine.appendLogicState("Trigger Shooter", new SingleLogicState(stateMachine) {

                    @Override
                    public void main(SensorData sensorData, HardwareData hardwareData) {
                        shot = true;
                    }
                });
                system.put("Move One", new TrueTimeTerminator(500));
                system.put("Trigger Shooter", new TrueTimeTerminator(100));
                system.put("Move Two", new TrueTimeTerminator(130));
                system.put("Trigger Shooter", new TrueTimeTerminator(100));
                system.put("Move Three", new TrueTimeTerminator(130));
                system.put("Trigger Shooter", new TrueTimeTerminator(200));
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                Vector2 targetPosition = this.targetPosition.clone();

                double deltaX = targetPosition.getA()-position.getA();
                double deltaY = targetPosition.getB()-position.getB();
                angDelta = MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY));
                if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack() && !gamepad1.a){
                    //rotOffset = Math.toDegrees(tmp - angDelta);
                    angDelta = Math.toRadians(hardware.smartDevices.get("SmartCV", SmartCV.class).getHeading());
                }else{
                    //angDelta = angDelta + Math.toRadians(rotOffset);
                    angDelta = 0;
                }

                double dtheta = angDelta - prevAng;
                double dt = ProgramClock.getFrameTimeMillis();
                turretTimer += Math.toDegrees(Math.toDegrees(dtheta)) * (750/270);
                if(turretTimer > 1000){
                    turretTimer = 1000;
                }
                turretTimer -= dt;
                if(turretTimer < -1){
                    turretTimer = -1;
                }
                prevAng = angDelta;

                double[] powershots = sensorData.getPowershots();
                if(gamepad1.b || shoot3){
                    angDelta = Math.toRadians(powershots[0]);
                }
                if(gamepad1.y || shoot2){
                    angDelta = Math.toRadians(powershots[1]);
                }
                if(gamepad1.x || shoot1){
                    angDelta = Math.toRadians(powershots[2]);
                }
                if(gamepad1.dpad_up){
                    system.update(sensorData, hardwareData);
                }else{
                    shoot1 = false;
                    shoot2 = false;
                    shoot3 = false;
                    system.reset();
                }

                if(gamepad2.right_bumper){
                    hardware.smartDevices.get("SmartCV", SmartCV.class).calibratePitch();
                }

                hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));

                DecimalFormat format = new DecimalFormat("#.##");

                telemetry.addData("Powershots", format.format(powershots[0]) + " " + format.format(powershots[1]) + " " + format.format(powershots[2]));
                telemetry.addData("Turning", hardwareData.getTurret());
                telemetry.addData("Heading", Math.toDegrees(angDelta));
                telemetry.addData("Range", sensorData.getRange());
                telemetry.addData("FPS", fps);
            }
        });

        eventSystem.onStart("Powershot Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.x){
                    //stateMachine.setActiveDriveState("LinearMove");
                }
                if(gamepad1.y || gamepad1.a){
                    if(stateMachine.driveStateActive("GamepadDrive")){
                        if(!stateMachine.logicStateActive("Load Shooter Turn")){
                            //stateMachine.deactivateState("Load Shooter");
                            //odometer.reset();
                            //stateMachine.activateLogic("Load Shooter Turn");
                        }
                    }
                }
                if(gamepad1.left_trigger > 0.25 || gamepad2.right_trigger > 0.25){
                    stateMachine.setActiveDriveState("Rotate To Target");
                }else{
                    timer=0;
                }
                if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2){
                    stateMachine.setActiveDriveState("GamepadDrive");
                    if(stateMachine.logicStateActive("Rotate To Target")){
                        stateMachine.deactivateState("Rotate To Target");
                    }
                }
                telemetry.addData("States", stateMachine.getActiveStates());
            }
        });

        eventSystem.onStart("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower((gamepad1.right_bumper ? 1 : (gamepad1.left_bumper ? -0.7 : 0)));
                hardwareData.setIntakePower((gamepad2.left_trigger > 0.1 ? 0 : hardwareData.getIntake()));

                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
            }
        });

        eventSystem.onStart("Shoot", new LogicState(stateMachine) {
            double tiltLevel = 0;
            long frameTime = System.currentTimeMillis();
            PIDFSystem system = new PIDFSystem(p, i, d, f);
            final double targetSpeed = 4.25;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (!gamepad2.left_bumper) ? 0.75 : 0;
                double targetSpeed = (!gamepad2.left_bumper) ? this.targetSpeed : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                system.setCoef(new Vector4(p, i, d, f));
                //telemetry.addData("Shooter Velocity", vel);
                hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel, (gamepad1.right_trigger > 0.2 ? 1 : 0)));
                if(stopShooter){
                    hardwareData.setShooter(0);
                }
                Telemetry t = FtcDashboard.getInstance().getTelemetry();
                t.addData("Speed", vel);
                t.addData("Target", targetSpeed);
                t.update();
                if(gamepad2.dpad_up){
                    //tiltLevel += (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.HOLD_INTAKE);
                }else if(gamepad2.dpad_down){
                    //tiltLevel -= (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                }else{
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.IDLE_INTAKE);
                }

                if(gamepad1.dpad_down){
                    if(!stateMachine.logicStateActive("Load Shooter")){
                        stateMachine.activateLogic("Load Shooter");
                    }
                }

                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);

                if(holdShoot){
                    hardwareData.setShooterTilt(0.335 + tiltLevel);
                }else{
                    //hardwareData.setShooterTilt(0.35 + tiltLevel);
                }

                if(Math.abs(gamepad2.left_stick_y) > 0.2){
                    hardwareData.setShooterTilt(0.49);
                    hardwareData.setShooter(0);
                    stopShooter = true;
                    holdShoot = false;
                }else if(gamepad2.right_stick_y < -0.2){
                    holdShoot = true;
                }else if(gamepad2.right_stick_y > 0.2 || gamepad1.y || gamepad1.b || gamepad1.x || gamepad1.dpad_up){
                    hardwareData.setShooterTilt(0.35 + tiltLevel);
                }

                if(gamepad2.a){
                    stopShooter = true;
                    hardwareData.setShooterTilt(0.49);
                }else if(Math.abs(gamepad2.left_stick_y) < 0.2){
                    stopShooter = false;
                }

                //telemetry.addData("Tilt", tiltLevel);
                //telemetry.addData("Servo", hardwareData.getShooterTilt());
                //hardwareData.setShooterTilt(tiltLevel);
                //telemetry.addData("Servo", "Shooter");
                frameTime = System.currentTimeMillis();
            }
        });

        eventSystem.onStart("Adjust Angle", new LogicState(stateMachine) {
            boolean lastRight = false, lastLeft = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad2.left_bumper && !lastLeft){
                    rotOffset += 1;
                }
                if(gamepad2.right_bumper && !lastRight){
                    rotOffset -= 1;
                }
                lastRight = gamepad2.right_bumper;
                lastLeft = gamepad2.left_bumper;
                telemetry.addData("Rot Offset", rotOffset);
            }
        });

        eventSystem.onStart("Wobble Control", new LogicState(stateMachine) {

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                 if(gamepad2.a){
                     //hardwareData.setWobbleLiftRight(0.43622);
                     //hardwareData.setWobbleLiftLeft(0.5208);
                 }
                 if(gamepad2.x){
                     hardwareData.setWobbleLiftRight(0.49055);
                     hardwareData.setWobbleLiftLeft(0.4801);
                 }
                 if(gamepad2.b){
                     hardwareData.setWobbleLiftRight(0.57676);
                     hardwareData.setWobbleLiftLeft(0.37882);
                 }
                 if(gamepad2.y){
                     hardwareData.setWobbleLiftRight(0.96634);
                     hardwareData.setWobbleLiftLeft(0.01);
                 }
                 double minVal = -1;
                 double maxVal = 1;
                 if(sensorData.getWobbleLift() >= 0){
                     maxVal = 0;
                 }
                 if(sensorData.getWobbleLift() <= -380){
                     minVal = 0;
                 }
                 maxVal = 0.2;
                 //telemetry.addData("Wobble", sensorData.getWobbleLift() + " | " + minVal + " | " + maxVal);
                //hardwareData.setWobbleLift(-MathUtils.clamp(gamepad2.left_stick_y * 1, minVal, maxVal));
                if(Math.abs(gamepad2.left_stick_y) < 0.1){
                     //hardwareData.setWobbleLift(0.1);
                 }
                 /**
                 if(gamepad2.left_stick_y < -0.1){
                     if(!stateMachine.logicStateActive("Lift Wobble")) {
                         stateMachine.activateLogic("Lift Wobble");
                     }
                     hardwareData.setWobbleLiftRight(0.57676);
                     hardwareData.setWobbleLiftLeft(0.37882);
                 }else if(gamepad2.left_stick_y > 0.1){
                     if(stateMachine.logicStateActive("Lift Wobble")){
                         stateMachine.deactivateState("Lift Wobble");
                     }
                     if(stateMachine.logicStateActive("Hold Wobble")){
                         stateMachine.deactivateState("Hold Wobble");
                     }
                     hardwareData.setWobbleLiftRight(0.43622);
                     hardwareData.setWobbleLiftLeft(0.5208);
                 }
                  */
                //telemetry.addData("Backlog", sensorData.getBacklog());
            }
        });

        stateMachine.appendLogicState("Lift Wobble", new LogicState(stateMachine) {
            double tolerence = 30, target = (390/2.0);
            double power = 0.8;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooterTilt(0.49);
                hardwareData.setWobbleLift(MathUtils.sign(target - (-sensorData.getWobbleLift())) * power);
                if(Math.abs(target - sensorData.getWobbleLift()) < tolerence){
                    stateMachine.activateLogic("Hold Wobble");
                    telemetry.addData("Moving", "Wobble");
                    deactivateThis();
                }
                telemetry.addData("Pos", sensorData.getWobbleLift());
            }
        });

        stateMachine.appendLogicState("Hold Wobble", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleLift(0.1);
            }
        });

        HashMap<String, LogicState> logicStates = new HashMap<>();
        eventSystem.onStart("Load Shooter", new LogicState(stateMachine) {
            int state = 3;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 80;
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
                        if(gamepad1.dpad_down) {
                            state = 0;
                        }else if(shot){
                            state = 0;
                            shot = false;
                        }
                    }
                }
                telemetry.addData("state", state);
            }
        });
        logicStates.put("Load Shooter Turn", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 175;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    //hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 175;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        stateMachine.setActiveDriveState("Rotate");
                        deactivateThis();
                    }
                }
                telemetry.addData("state", state);
            }
        });
        stateMachine.appendLogicStates(logicStates);
    }
}
