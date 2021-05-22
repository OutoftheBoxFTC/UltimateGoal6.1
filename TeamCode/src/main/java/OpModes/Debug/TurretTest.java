package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Hardware.*;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import MathSystems.Angle;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Motion.Terminators.OrientationTerminator;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
@Disabled
public class TurretTest extends BasicOpmode {
    Odometer odometer;
    Vector3 position, velocity;
    public TurretTest() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();

        position = Vector3.ZERO();
        velocity = Vector3.ZERO();

        odometer = new ConstantVOdometer(stateMachine, position, velocity);

        eventSystem.onStart("Odometry", odometer);

        eventSystem.onStart("Turret", new LogicState(stateMachine) {
            double angle = 0;
            boolean lastLeft = false, lastRight = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(angle));

                if(gamepad1.left_bumper && !lastLeft){
                    angle += 1;
                }
                if(gamepad1.right_bumper && !lastRight){
                    angle -= 1;
                }
                lastRight = gamepad1.right_bumper;
                lastLeft = gamepad1.left_bumper;
                telemetry.addData("Rot Offset", angle);

                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        });

        eventSystem.onStart("SpinShooter", new LogicState(stateMachine) {
            PIDSystem system = new PIDSystem(0.8, 0, 0);
            double target = 3.5, tilt = 0.33;
            boolean lastLeft = false, lastRight = false, lastdown = false, lastup = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                hardwareData.setShooter(0.7 + system.getCorrection(target - vel)); //0.75 | 4.5
                hardwareData.setShooterTilt(tilt);
                telemetry.addData("Vel", target);

                if(gamepad1.dpad_up && !lastLeft){
                    target += 0.1;
                }
                if(gamepad1.dpad_down && !lastRight){
                    target -= 0.1;
                }
                lastRight = gamepad1.dpad_down;
                lastLeft = gamepad1.dpad_up;


                if(gamepad1.dpad_right && !lastup){
                    tilt += 0.01;
                }
                if(gamepad1.dpad_left && !lastdown){
                    tilt -= 0.01;
                }
                lastdown = gamepad1.dpad_left;
                lastup = gamepad1.dpad_right;

                telemetry.addData("Target", target);
                telemetry.addData("Tilt", tilt);
                telemetry.addData("Pos", position);
            }
        });

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
                    timer = System.currentTimeMillis() + 120;
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        if(gamepad1.right_trigger > 0.25) {
                            state = 0;
                        }
                    }
                }
                telemetry.addData("state", state);
            }
        });
    }
}
