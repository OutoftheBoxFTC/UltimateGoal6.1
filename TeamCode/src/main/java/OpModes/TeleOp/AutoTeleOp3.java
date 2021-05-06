package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import Hardware.SmartDevices.SmartServo.SmartServo;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.ProgramClock;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Odometry.AdvancedVOdometer;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
@Config
public class AutoTeleOp3 extends BasicOpmode {
    AdvancedVOdometer odometer;
    Vector3 position, velocity;
    boolean holdShoot = true;
    boolean shot = false;
    boolean stopShooter = false;
    boolean posSet = false;
    int timer = 0;
    long turretTimer = 0;
    double rotOffset;
    public static double p = 1.5, i = 0, d = 0, f = 0.1;
    public AutoTeleOp3() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new AdvancedVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("odo", odometer);
        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Hit start to start");
                telemetry.addData("Singleton", SingletonVariables.getInstance().getPosition());
                odometer.setKinematicPosition(SingletonVariables.getInstance().getPosition());

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

        eventSystem.onStart("Turret", new LogicState(stateMachine) {
            double angDelta, prevAng;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                prevAng = 0;
                angDelta = 0;
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack() && !gamepad1.a && gamepad1.left_trigger > 0.1){
                    double[] pos = hardware.smartDevices.get("SmartCV", SmartCV.class).getPosition();
                    odometer.setKinematicPosition(pos[0], pos[1], 0);
                    posSet = true;
                }

                double deltaX = -(position.getA());
                double deltaY = -(position.getB());
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
                if(gamepad1.b){
                    angDelta = Math.toRadians(powershots[0]);
                }
                if(gamepad1.y){
                    angDelta = Math.toRadians(powershots[1]);
                }
                if(gamepad1.x){
                    angDelta = Math.toRadians(powershots[2]);
                }

                if(gamepad2.right_bumper){
                    hardware.smartDevices.get("SmartCV", SmartCV.class).calibratePitch();
                }

                hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(angDelta)));
                //hardware.smartDevices.get("Turret", SmartServo.class).forceSend(UGUtils.getTurretValue(Math.toDegrees(angDelta)));

                DecimalFormat format = new DecimalFormat("#.##");

                if(!posSet){
                    telemetry.addLine("WARNING! Kinematic Position is not set: Shooting estimation will be off!");
                }else{
                    telemetry.addData("Kinematic Position", position);
                    telemetry.addData("Velocity", velocity);
                }
            }
        });

        eventSystem.onStart("Powershot Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.left_trigger > 0.25 || gamepad2.right_trigger > 0.25){
                    shot = true;
                }else{
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
            final double targetSpeed = 4.25;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (!gamepad2.left_bumper) ? 0.75 : 0;
                double targetSpeed = (!gamepad2.left_bumper) ? this.targetSpeed : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                system.setCoef(new Vector4(p, i, d, f));
                hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel, (gamepad1.dpad_down ? 1 : 0)));
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
                    hardwareData.setShooterTilt(0.325);
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
                }else if(gamepad2.right_stick_y > 0.2 || gamepad1.y || gamepad1.b || gamepad1.x){
                    hardwareData.setShooterTilt(0.36);
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
                    timer = System.currentTimeMillis() + 80;
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
