package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.text.DecimalFormat;
import java.util.HashMap;

import Hardware.CustomClasses.SingletonVariables;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.ProgramClock;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.AdvancedVOdometer;
import OpModes.BasicOpmode;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@TeleOp
@Config
public class AutoTeleOp3 extends BasicOpmode {
    AdvancedVOdometer odometer, odometer2;
    Vector3 position, velocity, pos2, vel2;
    boolean holdShoot = true;
    boolean shot = false;
    boolean stopShooter = false;
    boolean posSet = false;
    boolean shoot1 = false, shoot2 = false, shoot3 = false;
    int timer = 0;
    long turretTimer = 0;
    double rotOffset;
    public static double p = 1, i = 0, d = 0, f = 0.1;
    public static double ARM_IDLE = 1400, ARM_DOWN = RobotConstants.UltimateGoal.INTAKE_BLOCKER_DOWN;
    public AutoTeleOp3() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        pos2 = Vector3.ZERO();
        vel2 = Vector3.ZERO();
        odometer2 = new AdvancedVOdometer(stateMachine, pos2, vel2);
        odometer = new AdvancedVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("odo", odometer);
        eventSystem.onStart("odo2", odometer2);
        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Hit start to start");
                telemetry.addData("Singleton", SingletonVariables.getInstance().getPosition());
                odometer.setKinematicPosition(SingletonVariables.getInstance().getPosition());

                hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                hardware.smartDevices.get("SmartCV", SmartCV.class).setPitchOffset(18.110011968744697);
                //hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES);

                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        eventSystem.onStart("GamepadDrive", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                double angVel = -gamepad1.right_stick_x;
                if(Math.abs(gamepad1.right_stick_x) > 0.1 || (Math.abs(gamepad1.left_stick_y) < 0.1 && Math.abs(gamepad1.left_stick_x) < 0.1)){
                    odometer2.reset();
                }else if(false){
                    double err = MathUtils.getRadRotDist(pos2.getC(), 0);
                    double maxStopVel = Math.sqrt(2 * RobotConstants.UltimateGoal.MAX_R_ACCEL * Math.abs(err));
                    angVel = MathUtils.sign(err) * Math.min(RobotConstants.UltimateGoal.MAX_ROTATION_SPEED, maxStopVel);
                    angVel = angVel / RobotConstants.UltimateGoal.MAX_ROTATION_SPEED;
                    angVel = angVel * 1.5;
                    //angVel = MathUtils.sign(err) * 0.5;
                    if(Math.abs(err) < Math.toRadians(5)){
                        angVel = 0;
                    }
                }
                double speedMod = (gamepad1.left_stick_button) ? 0.7 : 1;
                double rotSpeedMod = gamepad1.right_stick_button ? 0.7 : 1;
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, angVel * rotSpeedMod);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Pos2", pos2);
            }
        });

        eventSystem.onStart("Turret", new LogicState(stateMachine) {
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
                system.put("Move One", new TrueTimeTerminator(400));
                system.put("Trigger Shooter", new TrueTimeTerminator(100));
                system.put("Move Two", new TrueTimeTerminator(200));
                system.put("Trigger Shooter", new TrueTimeTerminator(100));
                system.put("Move Three", new TrueTimeTerminator(200));
                system.put("Trigger Shooter", new TrueTimeTerminator(200));
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack() && !gamepad1.a && (gamepad1.left_trigger > 0.1 || gamepad2.left_bumper) && velocity.getVector2().length() < 4){
                    double[] pos = hardware.smartDevices.get("SmartCV", SmartCV.class).getPosition();
                    odometer.setKinematicPosition(pos[0], pos[1], 0);
                    posSet = true;
                }

                double deltaX = -(position.getA());
                double deltaY = -(position.getB());

                if(velocity.getVector2().length() > 4){
                    double dist = hardware.smartDevices.get("SmartCV", SmartCV.class).getRange();
                    deltaX -= velocity.getA() * (dist / 120);
                    deltaY -= velocity.getB() * (dist / 120);
                }

                angDelta = MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY));

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

                hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));
                //hardware.smartDevices.get("Turret", SmartServo.class).forceSend(UGUtils.getTurretValue(Math.toDegrees(angDelta)));

                DecimalFormat format = new DecimalFormat("#.##");

                if(!posSet){
                    telemetry.addLine("WARNING! Kinematic Position is not set: Shooting estimation will be off!");
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                }else{
                    if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack()){
                        double odoTrack = MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY));
                        double cameraTrack = hardware.smartDevices.get("SmartCV", SmartCV.class).getRedHeading();
                        if(Math.abs(MathUtils.getRadRotDist(odoTrack, Math.toRadians(cameraTrack))) >= Math.toRadians(5)){
                            hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                        }else{
                            hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
                        }
                    }else{
                        hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON);
                    }
                    if(shot){
                        //hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE);
                    }
                    telemetry.addData("Kinematic Position", position);
                    telemetry.addData("Velocity", velocity);
                }
                telemetry.addData("Pattern", hardwareData.getPattern().toString());
            }
        });

        eventSystem.onStart("Powershot Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.left_trigger > 0.1){
                    shot = true;
                }else if(!gamepad1.dpad_up){
                    shot = false;
                    timer=0;
                }
                if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2){
                    stateMachine.setActiveDriveState("GamepadDrive");
                    if(stateMachine.logicStateActive("Rotate To Target")){
                        stateMachine.deactivateState("Rotate To Target");
                    }
                }
            }
        });

        eventSystem.onStart("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower((gamepad1.right_bumper ? 1 : (gamepad1.left_bumper ? -0.7 : 0)));
                hardwareData.setIntakePower((gamepad2.left_trigger > 0.1 ? 0 : hardwareData.getIntake()));

            }
        });

        eventSystem.onStart("Shoot", new LogicState(stateMachine) {
            long frameTime = System.currentTimeMillis();
            final PIDFSystem system = new PIDFSystem(p, i, d, f);
            final double targetSpeed = 4.5;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = 0.75;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                system.setCoef(new Vector4(p, i, d, f));
                if(gamepad1.dpad_up || gamepad1.b || gamepad1.x || gamepad1.y){
                    hardwareData.setShooter(0.7 + system.getCorrection(4.2 - vel, (gamepad1.left_trigger != 0 ? 1 : 0)));
                }else {
                    hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel, (gamepad1.dpad_down ? 1 : 0)));
                }
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
                if(gamepad2.right_bumper || gamepad1.right_trigger > 0.1){
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(ARM_DOWN));
                }else{
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(ARM_IDLE));
                }

                if(gamepad1.dpad_down){
                    if(!stateMachine.logicStateActive("Load Shooter")){
                        stateMachine.activateLogic("Load Shooter");
                    }
                }

                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);

                if(holdShoot){
                    hardwareData.setShooterTilt(0.334);
                }else{
                    //hardwareData.setShooterTilt(0.35 + tiltLevel);
                }

                if(Math.abs(gamepad2.left_stick_y) > 0.2){
                    //hardwareData.setShooterTilt(0.49);
                    //hardwareData.setShooter(0);
                    //stopShooter = true;
                    //holdShoot = false;
                }else if(gamepad2.right_stick_y < -0.2){
                    holdShoot = true;
                }else if(gamepad2.right_stick_y > 0.2 || gamepad1.y || gamepad1.b || gamepad1.x || gamepad1.dpad_up){
                    hardwareData.setShooterTilt(0.35);
                }

                if(gamepad2.a){
                    stopShooter = true;
                    hardwareData.setShooterTilt(0.49);
                }else if(Math.abs(gamepad2.left_stick_y) < 0.2){
                    stopShooter = false;
                }
                frameTime = System.currentTimeMillis();
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
                        if(gamepad1.dpad_down) {
                            state = 0;
                        }else if(shot){
                            state = 0;
                            shot = false;
                        }
                        hardwareData.setShooterLoadArm(0.925);
                    }
                    hardwareData.setShooterLoadArm(0.925);
                }
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
            }
        });
        stateMachine.appendLogicStates(logicStates);
    }
}
