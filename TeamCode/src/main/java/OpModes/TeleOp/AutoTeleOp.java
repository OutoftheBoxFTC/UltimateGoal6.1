package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

import Hardware.CustomClasses.SingletonVariables;
import Hardware.Hardware;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
public class AutoTeleOp extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean holdShoot = true;
    boolean shot = false;
    boolean stopShooter = false;
    int timer = 0;
    double rotOffset;
    public static Vector4 PIDF = new Vector4(0.8, 0, 0, 0.15);
    public AutoTeleOp() {
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
                telemetry.addData("Singleton", SingletonVariables.getInstance().getPosition());
                odometer.set(SingletonVariables.getInstance().getPosition());
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
                    return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, -gamepad1.right_stick_x * speedMod);
                }else{
                    if(gamepad1.left_trigger < 0.1) {
                        hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

                double deltaX = targetPosition.getA()-position.getA();
                double deltaY = targetPosition.getB()-position.getB();
                double angDelta = MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY)) + Math.toRadians(rotOffset);

                double maxSpeed = Math.sqrt(2 * RobotConstants.UltimateGoal.MAX_R_ACCEL * Math.abs(angDelta));
                maxSpeed = maxSpeed/RobotConstants.UltimateGoal.MAX_ROTATION_SPEED;
                maxSpeed = Math.max(maxSpeed, RobotConstants.UltimateGoal.KF);
                shot = timer > 10 && !gamepad1.dpad_up;
                if(Math.abs(angDelta) > Math.toRadians(1)){
                    timer = 0;
                    return new Vector3(0, 0, maxSpeed * MathUtils.sign(angDelta));
                }else {
                    timer += 1;
                    return Vector3.ZERO();
                }
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        eventSystem.onStart("Powershot Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.x){
                    stateMachine.setActiveDriveState("LinearMove");
                }
                if(gamepad1.y || gamepad1.a){
                    if(stateMachine.driveStateActive("GamepadDrive")){
                        if(!stateMachine.logicStateActive("Load Shooter Turn")){
                            stateMachine.deactivateState("Load Shooter");
                            odometer.reset();
                            stateMachine.activateLogic("Load Shooter Turn");
                        }
                    }
                }
                if(gamepad1.left_trigger > 0.25){
                    stateMachine.setActiveDriveState("Rotate To Target");
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
            final double targetSpeed = 4.75;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (gamepad2.right_trigger < 0.2) ? 0.75 : 0;
                double targetSpeed = (gamepad2.right_trigger < 0.2) ? this.targetSpeed : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                telemetry.addData("Shooter Velocity", vel);
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
                    hardwareData.setShooterTilt(0.33 + tiltLevel);
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
                }else if(gamepad2.right_stick_y > 0.2){
                    hardwareData.setShooterTilt(0.34 + tiltLevel);
                }

                if(Math.abs(gamepad2.left_trigger) > 0.2){
                    stopShooter = true;
                    hardwareData.setShooterTilt(0.49);
                }else if(Math.abs(gamepad2.left_stick_y) < 0.2){
                    stopShooter = false;
                }

                telemetry.addData("Tilt", tiltLevel);
                telemetry.addData("Servo", hardwareData.getShooterTilt());
                //hardwareData.setShooterTilt(tiltLevel);
                telemetry.addData("Servo", "Shooter");
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
                    hardwareData.setShooterLoadArm(0.875);
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
                    timer = System.currentTimeMillis() + 140;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    //hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 140;
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
