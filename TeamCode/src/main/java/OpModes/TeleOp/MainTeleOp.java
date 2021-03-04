package OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import MathSystems.MathUtils;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
public class MainTeleOp extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean holdShoot = true;
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
                    return new Vector3(gamepad1.left_stick_x * speedMod, gamepad1.left_stick_y * speedMod, -gamepad1.right_stick_x * speedMod);
                }else if(gamepad1.left_trigger > 0.1){
                    double speedMod = (gamepad1.left_trigger != 0) ? 0.6 : 1;
                    return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x * speedMod);
                }else{
                    return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
                }
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
            }
        });

        stateMachine.appendDriveState("Rotate", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                return new Vector3(0, 0, -0.2);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(MathUtils.getRadRotDist(position.getC(), Math.toRadians(4)) < Math.toRadians(1)){
                    stateMachine.setActiveDriveState("GamepadDrive");
                }
            }
        });

        eventSystem.onStart("Rotate Manager", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.a){
                    if(stateMachine.driveStateActive("GamepadDrive")){
                        if(!stateMachine.logicStateActive("Load Shooter Turn")){
                            odometer.reset();
                            stateMachine.activateLogic("Load Shooter Turn");
                        }
                    }
                }
                if(Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2){
                    if(stateMachine.driveStateActive("Rotate")){
                        stateMachine.setActiveDriveState("GamepadDrive");
                    }
                }
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
            PIDSystem system = new PIDSystem(1, 0, 0);
            final double targetSpeed = 5.0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (gamepad2.right_trigger > 0.2) ? 0.8 : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                telemetry.addData("Shooter Velocity", vel);
                if(gamepad2.right_trigger > 0.1) {
                    hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel));
                }else{
                    hardwareData.setShooter(0.2);
                }
                if(gamepad2.dpad_up){
                    tiltLevel += (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                }
                if(gamepad2.dpad_down){
                    tiltLevel -= (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                }

                if(gamepad2.right_bumper){
                    if(!stateMachine.logicStateActive("Load Shooter")){
                        stateMachine.activateLogic("Load Shooter");
                    }
                }

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
                    hardwareData.setShooterTilt(0.35 + tiltLevel);
                }

                telemetry.addData("Tilt", tiltLevel);
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
                    timer = System.currentTimeMillis() + 140;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.875);
                    timer = System.currentTimeMillis() + 140;
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
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 140;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.875);
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
        /**
         *
         */
        stateMachine.appendLogicStates(logicStates);
    }
}
