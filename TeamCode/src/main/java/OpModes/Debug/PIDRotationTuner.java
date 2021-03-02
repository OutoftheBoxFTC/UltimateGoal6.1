package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import Debug.Logger;
import Hardware.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathSystems.ConstantVMathUtil;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Motion.PIDDriveToPoint.PIDDriveToPoint;
import Motion.PIDDriveToPoint.PIDDriveToPointBuilder;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
public class PIDRotationTuner extends BasicOpmode {
    Vector3 position;
    Vector2 pos2d;
    Odometer odometer;
    public PIDRotationTuner() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        pos2d = Vector2.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, Vector3.ZERO());
        eventSystem.onInit("Server", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //Logger.getInstance().update();
            }
        });

        stateMachine.appendDriveState("Stop", new GamepadDriveState(stateMachine, gamepad1));

        eventSystem.onStart("Main", new LogicState(stateMachine) {
            double p = 1;
            long frameTime = System.currentTimeMillis();
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad1.left_bumper && !stateMachine.logicStateActive("Rotate")){
                    double tau = 2 * Math.PI;
                    stateMachine.setActiveDriveState("Rotate");
                }
                if(gamepad1.right_bumper){
                    stateMachine.deactivateState("Rotate");
                    stateMachine.setActiveDriveState("Stop");
                }
                if(gamepad1.dpad_up){
                    if(gamepad1.a)
                        p += ((gamepad1.left_trigger/10) * ((System.currentTimeMillis() - frameTime)/1000.0));

                }
                if(gamepad1.dpad_down){
                    if(gamepad1.a)
                        p -= ((gamepad1.left_trigger/10) * ((System.currentTimeMillis() - frameTime)/1000.0));
                }
                telemetry.addData("F", p);
                telemetry.addData("Control", gamepad1.left_trigger/10);
                telemetry.addData("A", Math.toDegrees(position.getC()));
                telemetry.addData("Drive", stateMachine.logicStateActive("Rotate"));
                frameTime = System.currentTimeMillis();
            }
        });

        eventSystem.onStart("Odometry", odometer);

    }
}
