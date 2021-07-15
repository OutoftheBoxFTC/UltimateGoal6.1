package OpModes.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import Odometry.AdvancedVOdometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@TeleOp
public class TensorTeleop extends BasicOpmode {
    AdvancedVOdometer odometer, trackingOdo;
    Vector3 position, velocity, timestampedPosition, trackingPos, trackingVel;
    long timestamp = 0, dataTimestamp = 0;
    TARGET turretTarget = TARGET.NONE;
    COLOR color = COLOR.RED;
    boolean shoot = false, shooterReady = false;
    double speedMod = 1;
    public static double ARM_IDLE = 1400, ARM_DOWN = RobotConstants.UltimateGoal.INTAKE_BLOCKER_DOWN;

    WOBBLE_FORK_POSITION forkPos = WOBBLE_FORK_POSITION.IN, targetForkPos = WOBBLE_FORK_POSITION.IN;
    WOBBLE_FOURBAR_POSITION fourbarPos = WOBBLE_FOURBAR_POSITION.IN, targetFourbarPos = WOBBLE_FOURBAR_POSITION.IN;

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
        trackingPos = Vector3.ZERO();
        trackingVel = Vector3.ZERO();
        timestampedPosition = Vector3.ZERO();
        odometer = new AdvancedVOdometer(stateMachine, position, velocity);
        trackingOdo = new AdvancedVOdometer(stateMachine, trackingPos, trackingVel);

        eventSystem.onInit("Odometry", odometer);
        eventSystem.onInit("TrackingVelOdo", trackingOdo);

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
                hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                hardware.smartDevices.get("SmartCV", SmartCV.class).setPitchOffset(29.695);
                hardware.smartDevices.get("SmartCV", SmartCV.class).setVelocity(velocity);
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
                telemetry.addData("Timestamped Pos", timestampedPosition);
                telemetry.addData("Pos", position);
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
            double goalOffset = 16.5;
            double powershotOffset = 7.5;
            double deltaX = 0, deltaY = 0;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                smartCV = hardware.getSmartDevices().get("SmartCV", SmartCV.class);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                switch(turretTarget){ //TODO: Ensure angle calculations still work
                    case BLUE_GOAL:
                        deltaX = -(position.getA()+35);
                        deltaY = -(position.getB());
                        break;
                    case RED_GOAL:
                        deltaX = -(position.getA()-35);
                        deltaY = -(position.getB());
                        break;
                    case RED_POWERSHOT_LEFT:
                        deltaX = -(position.getA() - (35 - goalOffset - powershotOffset - powershotOffset));
                        deltaY = -(position.getB());
                        break;
                    case RED_POWERSHOT_CENTER:
                        deltaX = -(position.getA() - (35 - goalOffset - powershotOffset));
                        deltaY = -(position.getB());
                        break;
                    case RED_POWERSHOT_RIGHT:
                        deltaX = -(position.getA() - (35 - goalOffset));
                        deltaY = -(position.getB());
                        break;
                    case BLUE_POWERSHOT_LEFT:
                        deltaX = -(position.getA() + (35 - goalOffset));
                        deltaY = -(position.getB());
                        break;
                    case BLUE_POWERSHOT_CENTER:
                        deltaX = -(position.getA() + (35 - goalOffset - powershotOffset));
                        deltaY = -(position.getB());
                        break;
                    case BLUE_POWERSHOT_RIGHT:
                        deltaX = -(position.getA() + (35 - goalOffset - powershotOffset - powershotOffset));
                        deltaY = -(position.getB());
                        break;
                    case NONE:
                        break;
                }
                if(velocity.getVector2().length() > 4){
                    double dist = hardware.smartDevices.get("SmartCV", SmartCV.class).getRange();
                    //deltaX -= velocity.getA() * (dist / 120);
                    //deltaY -= velocity.getB() * (dist / 120);
                }
                if(fourbarPos == WOBBLE_FOURBAR_POSITION.IN && targetFourbarPos == WOBBLE_FOURBAR_POSITION.IN) {
                    telemetry.addData("Angle", Math.toDegrees(MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY))));
                    hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(MathUtils.getRadRotDist(position.getC(), -Math.atan2(deltaX, deltaY)))));
                }
            }
        });

        eventSystem.onStart("Drive", new VelocityDriveState(stateMachine) {
            final double minSpeed = 0.1;
            final double minDist = 5;
            final double maxSpeed = 1;
            final double maxDist = 35;

            double speed = 1;

            final Vector3 lastPos = Vector3.ZERO();
            final PIDSystem x = new PIDSystem(1, 1, 0.2, 1);
            final PIDSystem y = new PIDSystem(1, 1, 0.2, 1);
            final PIDSystem r = new PIDSystem(0.1, 0.5, 0.2, 1);

            @Override
            public Vector3 getVelocities() {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
                    lastPos.set(trackingPos);
                    return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x).scale(speed).scale(speedMod);
                }else{
                    if(lastPos.getVector2().distanceTo(trackingPos.getVector2()) > 0.15) {
                        double xcorr = x.getCorrection(lastPos.getA() - trackingPos.getA());
                        double ycorr = y.getCorrection(lastPos.getB() - trackingPos.getB());
                        double rcorr = r.getCorrection(MathUtils.getRadRotDist(position.getC(), trackingPos.getC()));
                        return new Vector3(xcorr, ycorr, rcorr);
                    }
                }
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double slope = (maxSpeed - minSpeed) / (maxDist - minDist);
                double intercept = maxSpeed - (slope * maxDist);

                if(fourbarPos != WOBBLE_FOURBAR_POSITION.IN){
                    speed = Math.max((slope * sensorData.getDistance()) + intercept, minSpeed);
                }else{
                    speed = 1;
                }
            }
        });

        eventSystem.onStart("Driver", new LogicState(stateMachine) {
            boolean lastUp = false, lastDown = false;
            int state = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setIntakePower(gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0);
                //To simplify the code and changes, both Blue and Red modes are in one TeleOp and can be switched
                //With start and back
                if(gamepad1.back){
                    color = COLOR.BLUE;
                }
                if(gamepad1.start){
                    color = COLOR.RED;
                }
                //Powershot control
                //Currently on the dpad because its easier to press
                if(gamepad1.x){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_LEFT;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_LEFT;
                    }
                }else if(gamepad1.y){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_CENTER;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_CENTER;
                    }
                }else if(gamepad1.b){
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_POWERSHOT_RIGHT;
                    }else{
                        turretTarget = TARGET.RED_POWERSHOT_RIGHT;
                    }
                }else{
                    if(color == COLOR.BLUE){
                        turretTarget = TARGET.BLUE_GOAL;
                    }else{
                        turretTarget = TARGET.RED_GOAL;
                    }
                }
                //Shooting is not on a hair trigger because it has gotten "stuck" before, constantly returning ~0.1
                if(gamepad1.left_trigger > 0.15){
                    shoot = true;
                }
                if(gamepad1.right_trigger > 0.15){
                    speedMod = 0.5;
                }else{
                    speedMod = 1;
                }

                if(gamepad1.dpad_up && lastUp){
                    state ++;
                    if(state == 4){
                        state = 0;
                    }
                }
                if(gamepad1.dpad_down && lastDown){
                    state --;
                    if(state < 0){
                        state = 3;
                    }
                }

                if(state == 0){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.IN;
                    targetForkPos = WOBBLE_FORK_POSITION.IN;
                }else if(state == 1){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.IN;
                    targetForkPos = WOBBLE_FORK_POSITION.OUT;
                }else if(state == 2){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.TRAVEL;
                    targetForkPos = WOBBLE_FORK_POSITION.TRAVEL;
                }else if(state == 3){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.SCORE;
                    targetForkPos = WOBBLE_FORK_POSITION.OUT;
                }

                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;
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

                if(gamepad2.y){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.TRAVEL;
                    targetForkPos = WOBBLE_FORK_POSITION.TRAVEL;
                }
                if(gamepad2.a){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.IN;
                    targetForkPos = WOBBLE_FORK_POSITION.IN;
                }
                if(gamepad2.x){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.IN;
                    targetForkPos = WOBBLE_FORK_POSITION.OUT;
                }
                if(gamepad2.b){
                    targetFourbarPos = WOBBLE_FOURBAR_POSITION.SCORE;
                    targetForkPos = WOBBLE_FORK_POSITION.OUT;
                }
                telemetry.addData("Wobble", fourbarPos + " | " + forkPos + " | " + targetFourbarPos + " | " + targetForkPos);
            }
        });

        eventSystem.onStart("Shooter", new LogicState(stateMachine) {
            final PIDFSystem system = new PIDFSystem(3, 0, 0, 0.1);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //TODO: Put this velocity in SensorData
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                if(fourbarPos == WOBBLE_FOURBAR_POSITION.TRAVEL || fourbarPos == WOBBLE_FOURBAR_POSITION.SCORE || targetFourbarPos == WOBBLE_FOURBAR_POSITION.TRAVEL || targetFourbarPos == WOBBLE_FOURBAR_POSITION.SCORE){
                    hardwareData.setShooter(0 + system.getCorrection(0 - vel, 0));
                    hardwareData.setShooterTilt(0.42);
                    shooterReady = false;
                    return;
                }
                if(turretTarget == TARGET.BLUE_GOAL || turretTarget == TARGET.RED_GOAL || turretTarget == TARGET.NONE){
                    //Targeting the goal
                    hardwareData.setShooter(0.75 + system.getCorrection(4 - vel, shoot ? 1 : 0));
                    if(gamepad1.right_bumper || gamepad1.left_bumper){
                        hardwareData.setShooterTilt(0.355);
                    }else {
                        //hardwareData.setShooterTilt(0.37);
                    }
                    if(Math.abs(vel - 4) < 0.1){
                        shooterReady = true;
                    }else{
                        shooterReady = false;
                    }
                }else {
                    //Targeting the powershots
                    hardwareData.setShooter(0.7 + system.getCorrection(4.2 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.335);
                    if(Math.abs(vel - 4.2) < 0.1){
                        shooterReady = true;
                    }else{
                        //shooterReady = false;
                    }
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
                    hardwareData.setShooterLoadArm(0.985);
                    timer = System.currentTimeMillis() + 95; //Wait for indexer to move and shooter to grab ring
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2; //Waiting for the "in" move to complete...
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.7);//Retract the indexer out
                    timer = System.currentTimeMillis() + 100; //Wait for indexer to move and next ring to fall
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        if(shoot && shooterReady){//Wait for the next shoot command
                            shoot = false;//Set shoot to false since we are shooting the ring now
                            state = 0;
                        }
                        hardwareData.setShooterLoadArm(0.7); //Ensure the load arm is retracted
                    }
                    hardwareData.setShooterLoadArm(0.7); //Ensure the load arm is retracted
                }
            }
        });

        eventSystem.onStart("Change Fork Position", new LogicState(stateMachine) {
            int state = 0;
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(fourbarPos == targetFourbarPos && forkPos == targetForkPos){
                    return;
                }
                if(fourbarPos == WOBBLE_FOURBAR_POSITION.IN){
                    if(state == 0){
                        hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_CHANGE);
                        hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_CHANGE);
                        state = 1;
                        timer = System.currentTimeMillis() + 100;
                    }
                    if(state == 1 && System.currentTimeMillis() > timer){
                        if(targetForkPos == WOBBLE_FORK_POSITION.IN){
                            hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_IN);
                            hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_IN);
                        }else if(targetForkPos == WOBBLE_FORK_POSITION.OUT){
                            hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_OUT);
                            hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_OUT);
                        }else{
                            hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_TRAVEL);
                            hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_TRAVEL);
                        }
                        state = 2;
                        timer = System.currentTimeMillis() + 100;
                    }
                    if(state == 2 && System.currentTimeMillis() > timer){
                        if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.IN){
                            hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                            hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                        }else if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.TRAVEL){
                            hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_TRAVEL);
                            hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_TRAVEL);
                        }else if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.SCORE){
                            hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_SCORE);
                            hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_SCORE);
                        }
                        fourbarPos = targetFourbarPos;
                        forkPos = targetForkPos;
                        state = 0;
                    }
                }else{
                    if(targetForkPos == WOBBLE_FORK_POSITION.IN){
                        hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_IN);
                        hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_IN);
                    }else if(targetForkPos == WOBBLE_FORK_POSITION.OUT){
                        hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_OUT);
                        hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_OUT);
                    }else{
                        hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_TRAVEL);
                        hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_TRAVEL);
                    }

                    if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.IN){
                        hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                        hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                    }else if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.TRAVEL){
                        hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_TRAVEL);
                        hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_TRAVEL);
                    }else if(targetFourbarPos == WOBBLE_FOURBAR_POSITION.SCORE){
                        hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_SCORE);
                        hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_SCORE);
                    }

                    fourbarPos = targetFourbarPos;
                    forkPos = targetForkPos;
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

    public enum WOBBLE_FOURBAR_POSITION{
        IN,
        TRAVEL,
        SCORE
    }

    public enum WOBBLE_FORK_POSITION{
        OUT,
        IN,
        TRAVEL
    }
}
