package OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Hardware.Hardware;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.LogicState;

@TeleOp
public class TableTeleOp extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    public TableTeleOp() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        hardware.disableDevice(Hardware.HardwareDevices.DRIVE_MOTORS);
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

        eventSystem.onStart("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower((gamepad1.right_bumper ? 1 : 0));

                telemetry.addData("Position", position);
                telemetry.addData("Velocity", velocity);
                telemetry.addData("Left Shooter", hardware.getSmartDevices().get("Shooter Left", SmartMotor.class).getVelocity());
                telemetry.addData("Right Shooter", hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity());
            }
        });

        eventSystem.onStart("Shoot", new LogicState(stateMachine) {
            double tiltLevel = 0.37482;
            long frameTime = System.currentTimeMillis();
            PIDSystem system = new PIDSystem(0.7, 0, 0);
            final double targetSpeed = 5.0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double reqSpeed = (gamepad1.right_trigger != 0) ? 0.8 : 0;
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                telemetry.addData("Vel", vel);
                telemetry.addData("Corr", system.getCorrection(targetSpeed - vel));
                telemetry.addData("err", targetSpeed-vel);
                if(gamepad1.right_trigger != 0)
                    hardwareData.setShooter(reqSpeed + system.getCorrection(targetSpeed - vel));
                telemetry.addData("Speed", hardwareData.getShooter());
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

                telemetry.addData("Tilt", tiltLevel);
                if(gamepad2.right_trigger > 0.1){
                    //hardwareData.setWobbleLiftLeft(tiltLevel);
                    telemetry.addData("Servo", "Wobble Left");
                }else if(gamepad2.left_trigger > 0.1){
                    //hardwareData.setWobbleLiftRight(tiltLevel);
                    telemetry.addData("Servo", "Wobble Right");
                }else {
                    hardwareData.setShooterTilt(tiltLevel);
                    telemetry.addData("Servo", "Shooter");
                }
                frameTime = System.currentTimeMillis();
            }
        });

        eventSystem.onStart("Wobble Control", new LogicState(stateMachine) {

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

                if(gamepad1.a){
                    hardwareData.setWobbleFourbarRight(0.43622);
                    hardwareData.setWobbleFourbarLeft(0.5208);
                }
                if(gamepad1.x){
                    hardwareData.setWobbleFourbarRight(0.49055);
                    hardwareData.setWobbleFourbarLeft(0.4801);
                }
                if(gamepad1.b){
                    hardwareData.setWobbleFourbarRight(0.57676);
                    hardwareData.setWobbleFourbarLeft(0.37882);
                }
                if(gamepad1.y){
                    hardwareData.setWobbleFourbarRight(0.96634);
                    hardwareData.setWobbleFourbarLeft(0.01);
                }

                telemetry.addData("Backlog", sensorData.getBacklog());
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

        stateMachine.appendLogicStates(logicStates);
    }
}
