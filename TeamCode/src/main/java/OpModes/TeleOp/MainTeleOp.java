package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.GamepadDriveState;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
public class MainTeleOp extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean holdShoot = true;
    public static Vector4 PIDF = new Vector4(1, 0, 0, 1);
    public MainTeleOp() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        hardware.disableDevice(Hardware.HardwareDevices.WEBCAM);
        hardware.deRegisterDevice(Hardware.HardwareDevices.WEBCAM);
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("odo", odometer);
        eventSystem.onInit("Init", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Hit start to start");
                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        eventSystem.onStart("GamepadDrive", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                if(gamepad1.right_trigger > 0.1) {
                    double speedMod = (gamepad1.right_trigger != 0) ? 0.6 : 1;
                    hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, -gamepad1.right_stick_x * speedMod);
                }else if(gamepad1.left_trigger > 0.1){
                    double speedMod = (gamepad1.left_trigger != 0) ? 0.6 : 1;
                    hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, -gamepad1.right_stick_x * speedMod);
                }else{
                    hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
                return new Vector3(0.6, 0, 0);
            }

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                shot1 = System.currentTimeMillis() + 750;
                shot2 = shot1 + 400;
                shot3 = shot2 + 400;
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

        eventSystem.onStart("Powershot Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.x){
                    stateMachine.setActiveDriveState("LinearMove");
                }
                if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2){
                    stateMachine.setActiveDriveState("GamepadDrive");
                    if(stateMachine.logicStateActive("LinearMove")){
                        stateMachine.deactivateState("LinearMove");
                    }
                }
                telemetry.addData("States", stateMachine.getActiveStates());
            }
        });

        eventSystem.onStart("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower((gamepad1.right_bumper ? 1 : (gamepad1.left_bumper ? -1 : 0)));

                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
            }
        });

        eventSystem.onStart("Shoot", new LogicState(stateMachine) {
            double tiltLevel = 0;
            long frameTime = System.currentTimeMillis();
            PIDFSystem system = new PIDFSystem(PIDF);
            final double targetSpeed = 4.6875;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (gamepad2.right_trigger > 0.2) ? 0.75 : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                telemetry.addData("Shooter Velocity", vel);
                if(gamepad2.right_trigger > 0.1) {
                    hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel, (gamepad2.right_bumper ? 1 : 0)));
                }else{
                    hardwareData.setShooter(0.2);
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

                if(gamepad2.right_bumper){
                    if(!stateMachine.logicStateActive("Load Shooter")){
                        stateMachine.activateLogic("Load Shooter");
                    }
                }

                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);

                if(holdShoot){
                    hardwareData.setShooterTilt(0.345 + tiltLevel);
                }else{
                    //hardwareData.setShooterTilt(0.35 + tiltLevel);
                }

                if(Math.abs(gamepad2.left_stick_y) > 0.2){
                    hardwareData.setShooterTilt(0.49);
                    hardwareData.setShooter(0);
                    holdShoot = false;
                }else if(gamepad2.right_stick_y < -0.2){
                    holdShoot = true;
                }else if(gamepad2.right_stick_y > 0.2){
                    hardwareData.setShooterTilt(0.36 + tiltLevel);
                }

                telemetry.addData("Tilt", tiltLevel);
                telemetry.addData("Servo", hardwareData.getShooterTilt());
                //hardwareData.setShooterTilt(tiltLevel);
                telemetry.addData("Servo", "Shooter");
                frameTime = System.currentTimeMillis();
            }
        });

        eventSystem.onStart("Wobble Control", new LogicState(stateMachine) {

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                 if(gamepad2.a){
                     hardwareData.setWobbleLiftRight(0.43622);
                     hardwareData.setWobbleLiftLeft(0.5208);
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
                 telemetry.addData("Wobble", sensorData.getWobbleLift() + " | " + minVal + " | " + maxVal);
                hardwareData.setWobbleLift(-MathUtils.clamp(gamepad2.left_stick_y * 1, minVal, maxVal));
                if(Math.abs(gamepad2.left_stick_y) < 0.1){
                     hardwareData.setWobbleLift(0.1);
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
                telemetry.addData("Backlog", sensorData.getBacklog());
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
        logicStates.put("Load Shooter", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 70;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 70;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        if(gamepad2.right_bumper) {
                            state = 0;
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
            public void init(SensorData sensorData, HardwareData hardwareData) {
                state = 0;
                timer = 0;
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 70;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 70;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        deactivateThis();
                    }
                }
                telemetry.addData("state", state);
            }
        });
        stateMachine.appendLogicStates(logicStates);
    }
}
