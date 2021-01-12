package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SkystoneHardware;
import Hardware.SmartDevices.SmartEncoder.SmartEncoder;
import Hardware.UltimateGoalHardware;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
public class RobotTuner extends BasicOpmode {

    public RobotTuner() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        eventSystem.onStart("drive", new GamepadDriveState(stateMachine, gamepad1));
        eventSystem.onInit("Init Message", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("Click start to begin tuning, this will take a while. Make sure to note down every value accurately!");
                if(isStarted()){
                    deactivateThis();
                }
            }
        });
        eventSystem.onStart("Pods", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Left", sensorData.getOdometryLeft());
                telemetry.addData("Right", sensorData.getOdometryRight());
                telemetry.addData("Aux", sensorData.getOdometryAux());
            }
        });
        eventSystem.onStart("Encoder PPR Explanation", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Encoder PPR");
                telemetry.addLine("Rotate each encoder one full revolution and note down the encoder values and error");
                telemetry.addLine("This section tests the number of encoder pulses per full revolution of the encoder");
                telemetry.addLine("Press A to start...");
                if(gamepad1.a){
                    stateMachine.activateLogic("Encoder PPR Test");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("Encoder PPR Test", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Encoder PPR");
                telemetry.addData("Left Encoder", sensorData.getOdometryLeft() + " pulses | Error: " + Math.abs(sensorData.getOdometryLeft() - 4096));
                telemetry.addData("Right Encoder", sensorData.getOdometryRight() + " pulses | Error: " + Math.abs(sensorData.getOdometryRight() - 4096));
                telemetry.addData("Aux Encoder", sensorData.getOdometryAux() + " pulses | Error: " + Math.abs(sensorData.getOdometryAux() - 4096));
                telemetry.addLine("Press X when all values have been recorded");
                if(gamepad1.x){
                    stateMachine.activateLogic("Translation Factor Explanation");
                    hardware.getSmartDevices().get("Odometry Left", SmartEncoder.class).calibrate();
                    hardware.getSmartDevices().get("Odometry Right", SmartEncoder.class).calibrate();
                    hardware.getSmartDevices().get("Odometry Aux", SmartEncoder.class).calibrate();
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("Translation Factor Explanation", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Translation Factor");
                telemetry.addLine("Move the robot 48 inches forward and record the values and error");
                telemetry.addLine("This section tests the expected translation factor, the number of encoder ticks per inch");
                telemetry.addLine("Press A to start...");
                if(gamepad1.a){
                    stateMachine.activateLogic("Translation Factor");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("Translation Factor", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Translation Factor");
                telemetry.addData("Left Encoder", (sensorData.getOdometryLeft() / 48.0) + " | " + Math.abs(48.0 - (sensorData.getOdometryLeft() * 0.0011)) + " in error");
                telemetry.addData("Right Encoder", (sensorData.getOdometryRight() / 48.0) + " | " + Math.abs(48.0 - (sensorData.getOdometryRight() * 0.0011)) + " in error");
                telemetry.addData("Combined Encoder", (((sensorData.getOdometryRight() + sensorData.getOdometryLeft())/2.0) / 48.0) + " | " + Math.abs(48.0 - (((sensorData.getOdometryRight() + sensorData.getOdometryLeft())/2.0) * 0.0011)) + " in error");
                telemetry.addLine("Press X when all values have been recorded");
                if(gamepad1.x){
                    stateMachine.activateLogic("Track Width Explanation");
                    hardware.getSmartDevices().get("Odometry Left", SmartEncoder.class).calibrate();
                    hardware.getSmartDevices().get("Odometry Right", SmartEncoder.class).calibrate();
                    hardware.getSmartDevices().get("Odometry Aux", SmartEncoder.class).calibrate();
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("Track Width Explanation", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Track Width");
                telemetry.addLine("Rotate the robot 30 times and return the robot EXACTLY back to the starting angle and record the values and error");
                telemetry.addLine("This section tests the track width of the robot, the distance between the two pods used to calculate heading");
                telemetry.addLine("The section will also note down the aux pod track width factor");
                telemetry.addLine("Press A to start...");
                if(gamepad1.a){
                    stateMachine.activateLogic("Track Width");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("Track Width", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Section", "Track Width");
                double factor = ((sensorData.getOdometryRight() - sensorData.getOdometryLeft())/2.0)/(2 * 30 * Math.PI);
                telemetry.addData("Factor", factor + " | Error: " + (7149.42 - factor));
                telemetry.addData("Estimated Angle", Math.toDegrees(((sensorData.getOdometryRight() - sensorData.getOdometryLeft())/2.0) / factor));
                telemetry.addData("Aux Factor", (sensorData.getOdometryAux()/(2 * 30 * Math.PI)));
                telemetry.addLine("Press X when all values have been recorded");
                if(gamepad1.x){
                    stateMachine.activateLogic("End");
                    deactivateThis();
                }
            }
        });
        stateMachine.appendLogicState("End", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addLine("That's it! If the data is too different (~5% to 10% off), update the new values in the corresponding hardware class");
            }
        });
    }
}
