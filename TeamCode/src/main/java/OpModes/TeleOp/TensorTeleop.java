package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartCV.TowerGoal.TensorPipeline;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.PIDFSystem;
import MathSystems.Vector2;
import MathSystems.Vector3;
import MathSystems.Vector4;
import Odometry.AdvancedVOdometer;
import OpModes.BasicOpmode;
import State.DriveState;
import State.EventSystem.LinearEventSystem;
import State.GamepadDriveState;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@TeleOp
public class TensorTeleop extends BasicOpmode {
    AdvancedVOdometer odometer;
    Vector3 position, velocity, timestampedPosition;
    long timestamp = 0, dataTimestamp = 0;
    TARGET turretTarget = TARGET.NONE;
    COLOR color = COLOR.RED;
    boolean shoot = false;
    public static double ARM_IDLE = 1400, ARM_DOWN = RobotConstants.UltimateGoal.INTAKE_BLOCKER_DOWN;
    public TensorTeleop() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();

        //BE CAREFUL!
        //This position, velocity, and odometer are NOT field accurate
        //They give accurate angles to anything on the goal wall, but should NOT be used for field position
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        timestampedPosition = Vector3.ZERO();
        odometer = new AdvancedVOdometer(stateMachine, position, velocity);

        eventSystem.onInit("Odometry", odometer);

        eventSystem.onInit("Setup Colour", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //To simplify the code and changes, both Blue and Red modes are in one TeleOp and can be switched
                //With x and b (blue and red ofc)
                if(gamepad1.x){
                    color = COLOR.BLUE;
                }
                if(gamepad1.b){
                    color = COLOR.RED;
                }
                if(color == COLOR.RED){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                }else if (color == COLOR.BLUE){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                }
                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        eventSystem.onInit("Setup", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                gamepad1.setJoystickDeadzone(0.05f);
                //For some STUPID reason, the FIRST sdk sets the joystick deadzone
                //to like 20% of the joysticks movement by default
                //TODO: Fine tune this further
            }
        });

        eventSystem.onStart("Wobble Release", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
            }
        });

        eventSystem.onStart("Monitoring", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Target", turretTarget);
                telemetry.addData("Color", color);
            }
        });

        eventSystem.onInit("Vision", new LogicState(stateMachine) {
            SmartCV smartCV;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                smartCV = hardware.getSmartDevices().get("SmartCV", SmartCV.class);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //This timestamp is updated BEFORE the AI runs, so we grab our position and save it
                if(smartCV.getTimestamp() != timestamp){
                    timestamp = smartCV.getTimestamp();
                    timestampedPosition.set(position);
                }
                //This timestamp is updated AFTER the AI runs
                if(smartCV.getDataTimestamp() != dataTimestamp){
                    dataTimestamp = smartCV.getDataTimestamp();
                    //Calculate how much we have moved since the AI started running
                    Vector3 posdiff = position.subtract(timestampedPosition);
                    //Retroactively set and update our position, by adding the delta to our AI position
                    //Because the AI position will be old (~650ms behind)
                    Vector3 newpos = new Vector3(smartCV.getPosition()[0], smartCV.getPosition()[1], 0);
                    odometer.setKinematicPosition(newpos.add(posdiff));
                }
            }
        });

        eventSystem.onStart("Turret", new LogicState(stateMachine) {
            SmartCV smartCV;
            double angle = 0;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                smartCV = hardware.getSmartDevices().get("SmartCV", SmartCV.class);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                switch(turretTarget){ //TODO: Ensure angle calculations still work
                    case BLUE_GOAL:
                        //Check if we are moving faster then 2 inches per second
                        if(velocity.getVector2().length() > 2) {
                            //If we are moving, we need to track the high goal while keeping velocity in mind
                            //This vector is our delta to the goal
                            Vector2 v1 = position.getVector2().subtract(new Vector2(35, 0)).scale(-1);
                            //Then we subtract from the position our velocity times flight time (range / 120.0 in/s)
                            //Then we find the angle to zero to find our relative angle
                            angle = v1.subtract(velocity.getVector2().scale(smartCV.getRange()/120.0)).angleTo(Vector2.ZERO());
                        }else {
                            //If we are moving less then 2 inches per second chances are the ring will just go into the goal
                            //So we just calculate the angle to the goal
                            angle = position.getVector2().subtract(new Vector2(35, 0)).scale(-1).angleTo(Vector2.ZERO());
                        }
                        break;
                    case RED_GOAL:
                        //See above for more explanations
                        if(velocity.getVector2().length() > 2){
                            Vector2 v1 = position.getVector2().subtract(new Vector2(35, 0)).scale(-1);
                            angle = v1.subtract(velocity.getVector2().scale(smartCV.getRange()/120.0)).angleTo(Vector2.ZERO());
                        }else {
                            angle = position.getVector2().add(new Vector2(35, 0)).scale(-1).angleTo(Vector2.ZERO());
                        }
                        break;
                    case RED_POWERSHOT_LEFT:
                        angle = smartCV.getRedPowershots()[2];
                        break;
                    case RED_POWERSHOT_CENTER:
                        angle = smartCV.getRedPowershots()[1];
                        break;
                    case RED_POWERSHOT_RIGHT:
                        angle = smartCV.getRedPowershots()[0];
                        break;
                    case BLUE_POWERSHOT_LEFT:
                        angle = smartCV.getBluePowershots()[0];
                        break;
                    case BLUE_POWERSHOT_CENTER:
                        angle = smartCV.getBluePowershots()[1];
                        break;
                    case BLUE_POWERSHOT_RIGHT:
                        angle = smartCV.getBluePowershots()[2];
                        break;
                    case NONE:
                        break;
                }
                hardwareData.setTurret(UGUtils.getTurretValue(angle));
            }
        });

        eventSystem.onStart("Drive", new VelocityDriveState(stateMachine) {
            /**
             * Technically
             * @see GamepadDriveState
             * does this for us, but we write it out here in case we want to do anything fancy
             * Like velocity corrected drive, absolute breaking to prevent pushing
             * or to keep the heading constant while strafing
             */
            @Override
            public Vector3 getVelocities() {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        eventSystem.onStart("Driver", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);
                //To simplify the code and changes, both Blue and Red modes are in one TeleOp and can be switched
                //With x and b (blue and red ofc)
                if(gamepad1.x){
                    color = COLOR.BLUE;
                }
                if(gamepad1.b){
                    color = COLOR.RED;
                }
                //Powershot control
                //Currently on the dpad because its easier to press
                if(gamepad1.dpad_left){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_LEFT;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_LEFT;
                    }
                }
                if(gamepad1.dpad_up){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_CENTER;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_CENTER;
                    }
                }
                if(gamepad1.dpad_right){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_RIGHT;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_RIGHT;
                    }
                }
                //Shooting is not on a hair trigger because it has gotten "stuck" before, constantly returning ~0.1
                if(gamepad1.left_trigger > 0.15){
                    shoot = true;
                }
                if(gamepad1.right_trigger > 0.1){
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(ARM_DOWN));
                }else{
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(ARM_IDLE));
                }
            }
        });

        eventSystem.onStart("Operator", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(gamepad2.dpad_up){
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.HOLD_INTAKE);
                }else if(gamepad2.dpad_down){
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.RELEASE_INTAKE);
                }else{
                    hardwareData.setIntakeRelease(RobotConstants.UltimateGoal.IDLE_INTAKE);
                }
            }
        });

        eventSystem.onStart("Shooter", new LogicState(stateMachine) {
            final PIDFSystem system = new PIDFSystem(1, 0, 0, 0.1);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //TODO: Put this velocity in SensorData
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                if(turretTarget == TARGET.BLUE_GOAL || turretTarget == TARGET.RED_GOAL || turretTarget == TARGET.NONE){
                    //Targeting the goal
                    hardwareData.setShooter(0.75 + system.getCorrection(4.5 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.334);
                }else {
                    //Targeting the powershots
                    hardwareData.setShooter(0.7 + system.getCorrection(4.2 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.35);
                }
            }
        });

        eventSystem.onStart("Load Shooter", new LogicState(stateMachine) {
            int state = 3;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(state == 0){
                    //First we move the indexer into the hopper
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 100; //Wait for indexer to move and shooter to grab ring
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;//Waiting for the "in" move to complete...
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.925);//Retract the indexer out
                    timer = System.currentTimeMillis() + 100; //Wait for indexer to move and next ring to fall
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        if(shoot){//Wait for the next shoot command
                            shoot = false;//Set shoot to false since we are shooting the ring now
                            state = 0;
                        }
                        hardwareData.setShooterLoadArm(0.925); //Ensure the load arm is retracted
                    }
                    hardwareData.setShooterLoadArm(0.925); //Ensure the load arm is retracted
                }
            }
        });
    }

    public enum TARGET{
        BLUE_GOAL,
        RED_GOAL,
        RED_POWERSHOT_LEFT,
        RED_POWERSHOT_CENTER,
        RED_POWERSHOT_RIGHT,
        BLUE_POWERSHOT_LEFT,
        BLUE_POWERSHOT_CENTER,
        BLUE_POWERSHOT_RIGHT,
        NONE
    }

    public enum COLOR{
        BLUE,
        RED
    }
}
