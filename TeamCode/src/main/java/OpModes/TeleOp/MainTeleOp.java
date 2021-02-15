package OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.Vector3;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
public class MainTeleOp extends BasicOpmode {
    ConstantVOdometer odometer;
    Vector3 position, velocity;
    public MainTeleOp() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
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
        eventSystem.onStart("Drive", new GamepadDriveState(stateMachine, gamepad1));

        eventSystem.onStart("Intake", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower((gamepad1.right_bumper ? 1 : 0));

            }
        });

        eventSystem.onStart("Shoot", new LogicState(stateMachine) {
            double tiltLevel = 0.37482;
            long frameTime = System.currentTimeMillis();
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setShooter((gamepad1.right_trigger != 0) ? 0.8 : 0);
                if(gamepad2.dpad_up){
                    tiltLevel += (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                }
                if(gamepad2.dpad_down){
                    tiltLevel -= (0.01 * ((System.currentTimeMillis() - frameTime)/1000.0));
                }

                if(gamepad1.right_bumper){
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
                    hardwareData.setWobbleLiftRight(0.43622);
                    hardwareData.setWobbleLiftLeft(0.5208);
                }
                if(gamepad1.x){
                    hardwareData.setWobbleLiftRight(0.49055);
                    hardwareData.setWobbleLiftLeft(0.4801);
                }
                if(gamepad1.b){
                    hardwareData.setWobbleLiftRight(0.57676);
                    hardwareData.setWobbleLiftLeft(0.37882);
                }
                if(gamepad1.y){
                    hardwareData.setWobbleLiftRight(0.96634);
                    hardwareData.setWobbleLiftLeft(0.01);
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
                    hardwareData.setShooterLoadArm(0.5);
                    timer = System.currentTimeMillis() + 50;
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.3);
                    timer = System.currentTimeMillis() + 50;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        deactivateThis();
                    }
                }
            }
        });

        stateMachine.appendLogicStates(logicStates);
    }
}
