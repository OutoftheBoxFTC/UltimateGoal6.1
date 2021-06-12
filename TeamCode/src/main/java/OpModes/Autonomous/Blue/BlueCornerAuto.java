package OpModes.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Hardware.*;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.Vector3;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.TimeTerminator;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import OpModes.TeleOp.TensorTeleop;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@Autonomous
public class BlueCornerAuto extends BasicOpmode {
    long START_DELAY_TIME_MS = 1000;
    long DELAY_TIME_MS = 1000;

    ConstantVOdometer odometer;
    Vector3 position, velocity;
    boolean doStarterStack = true, pickupSecondWobble = false;
    DELAY_LOCATION delayLocation = DELAY_LOCATION.NO_DELAY;
    int startingStack = 0;
    LinearEventSystem linearSystem;
    CrosstrackBuilder builder;
    boolean shooterReady = false, shoot = false;
    TensorTeleop.TARGET turretTarget = TensorTeleop.TARGET.NONE;
    public BlueCornerAuto() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();

        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);

        linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);

        builder = new CrosstrackBuilder(stateMachine, position);

        eventSystem.onStart("Odometer", odometer);

        eventSystem.onInit("Monitoring", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("States", stateMachine.getActiveStates());
            }
        });

        eventSystem.onInit("Setup", new LogicState(stateMachine) {
            int idx = 0;
            boolean pressedDown = false, pressedUp = false, pressedLeft = false, pressedRight = false;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Num Rings", startingStack);
                telemetry.addLine("Setup");
                telemetry.addLine("=====================");
                if(gamepad1.dpad_up && !pressedUp){
                    idx --;
                    if(idx < 0){
                        idx = 3;
                    }
                }
                if(gamepad1.dpad_down && !pressedDown){
                    idx ++;
                    if(idx > 3){
                        idx = 0;
                    }
                }

                if(idx == 0){
                    telemetry.addData("Do Starting Stack", "<" + doStarterStack + ">");
                    if((gamepad1.dpad_left && !pressedLeft) || (gamepad1.dpad_right && !pressedRight)){
                        doStarterStack = !doStarterStack;
                    }
                }else{
                    telemetry.addData("Do Starting Stack: ", doStarterStack);
                }

                if(idx == 1){
                    telemetry.addData("Wait Location", "<" + delayLocation + ">");
                    if(gamepad1.dpad_left && !pressedLeft){
                        delayLocation = delayLocation.previous();
                    }
                    if(gamepad1.dpad_right && !pressedRight){
                        delayLocation = delayLocation.next();
                    }
                }else{
                    telemetry.addData("Wait Location", delayLocation);
                }

                if(idx == 2){
                    telemetry.addData("Start Delay Time", "<" + START_DELAY_TIME_MS + ">");
                    if(gamepad1.dpad_right && !pressedRight){
                        START_DELAY_TIME_MS += 250;
                    }
                    if(gamepad1.dpad_left && !pressedLeft){
                        START_DELAY_TIME_MS -= 250;
                        if(START_DELAY_TIME_MS < 0){
                            START_DELAY_TIME_MS = 0;
                        }
                    }
                }else{
                    telemetry.addData("Start Delay Time", START_DELAY_TIME_MS);
                }

                if(idx == 3){
                    telemetry.addData("Delay Time", "<" + DELAY_TIME_MS + ">");
                    if(gamepad1.dpad_right && !pressedRight){
                        DELAY_TIME_MS += 250;
                    }
                    if(gamepad1.dpad_left && !pressedLeft){
                        DELAY_TIME_MS -= 250;
                        if(DELAY_TIME_MS < 0){
                            DELAY_TIME_MS = 0;
                        }
                    }
                }else{
                    telemetry.addData("Delay Time", DELAY_TIME_MS);
                }

                if(idx == 4){
                    telemetry.addData("Pickup Second Wobble", "<" + pickupSecondWobble + ">");
                    if((gamepad1.dpad_left && !pressedLeft) || (gamepad1.dpad_right && !pressedRight)){
                        pickupSecondWobble = !pickupSecondWobble;
                    }
                }else{
                    //telemetry.addData("Pickup Second Wobble", pickupSecondWobble);
                }

                pressedDown = gamepad1.dpad_down;
                pressedUp = gamepad1.dpad_up;
                pressedLeft = gamepad1.dpad_left;
                pressedRight = gamepad1.dpad_right;
            }
        });

        eventSystem.onInit("Vision", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                startingStack = (int) sensorData.getRings();
                hardwareData.setIntakeShield(RobotConstants.UltimateGoal.INTAKE_BLOCKER_UP);
                hardwareData.setWobbleOneuseLeft(RobotConstants.UltimateGoal.ONEUSE_LEFT_ARM_HOLD);
                if(isStarted()){
                    deactivateThis();
                }
            }
        });

        stateMachine.appendLogicState("Stop", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        eventSystem.onStart("Setup Systems", new SingleLogicState(stateMachine) {
            Path highgoalPath, startingStackPath, wait1Path, wobblePath, wait2Path, secondWobblePath, parkPath;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                highgoalPath = new PathBuilder(0, 0, Angle.degrees(0)).lineTo(9, 13).complete();
                startingStackPath = new PathBuilder(highgoalPath.getEndpoint()).lineTo(21, 35).lineTo(21, 45).complete();
                hardwareData.setTurret(UGUtils.getTurretValue(-1.5));
            }

            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.put("Stop", new TrueTimeTerminator(START_DELAY_TIME_MS));

                linearSystem.put("Highgoal Shoot Pos", builder.follow(highgoalPath), new OrientationTerminator(position, highgoalPath));

                linearSystem.put("Shoot", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        shoot = true;
                    }
                }, new TrueTimeTerminator(1000));

                linearSystem.put("Stop", new TrueTimeTerminator(500));
                if(doStarterStack){
                    linearSystem.put("Intake", new VelocityDriveState(stateMachine) {
                        @Override
                        public Vector3 getVelocities() {
                            return Vector3.ZERO();
                        }

                        @Override
                        public void update(SensorData sensorData, HardwareData hardwareData) {
                            hardwareData.setIntakePower(1);
                        }
                    }, new TimeTerminator(2));

                    linearSystem.put("ShootStack", new VelocityDriveState(stateMachine) {
                        @Override
                        public Vector3 getVelocities() {
                            return Vector3.ZERO();
                        }

                        @Override
                        public void update(SensorData sensorData, HardwareData hardwareData) {
                            shoot = true;
                            hardwareData.setTurret(UGUtils.getTurretValue(7.5));
                        }
                    }, new TimeTerminator(2));

                    linearSystem.put("Starting Stack Shoot", builder.follow(startingStackPath, 2, 0.4, 0.3), new OrientationTerminator(position, startingStackPath));

                    linearSystem.put("ShootStack", new TimeTerminator(2));
                }
                wait1Path = new PathBuilder(doStarterStack ? startingStackPath.getEndpoint() : highgoalPath.getEndpoint()).lineTo(-5, 30).complete();
                linearSystem.put("Drive Wait 1", builder.follow(wait1Path, 2, 0.4, 0.3), new OrientationTerminator(position, wait1Path));

                if(delayLocation == DELAY_LOCATION.FIRST_LOCATION || delayLocation == DELAY_LOCATION.BOTH_LOCATIONS){
                    linearSystem.put("Stop", new TrueTimeTerminator(5000));
                }

                PathBuilder wobbleBuilder = new PathBuilder(wait1Path.getEndpoint());
                if(startingStack == 0){
                    wobblePath = wobbleBuilder.lineTo(0, 75).complete();
                }else if(startingStack == 1){
                    wobblePath = wobbleBuilder.lineTo(28, 99).complete();
                }else{
                    wobblePath = wobbleBuilder.lineTo(0, 123).complete();
                }
                linearSystem.put("Wobble Path", builder.follow(wobblePath), new OrientationTerminator(position, wobblePath));

                linearSystem.put("Deploy Oneuse", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        hardwareData.setWobbleOneuseLeft(RobotConstants.UltimateGoal.ONEUSE_LEFT_ARM_RELEASE);
                    }
                }, new TrueTimeTerminator(500));

                wait2Path = new PathBuilder(wobblePath.getEndpoint()).lineTo(21, 124, Angle.degrees(-90)).lineTo(45, 124, Angle.degrees(-90)).complete();
                linearSystem.put("Wait 2 Path", builder.follow(wait2Path, 2, 0.6, 0.15), new OrientationTerminator(position, wait2Path));

                if(delayLocation == DELAY_LOCATION.SECOND_LOCATION || delayLocation == DELAY_LOCATION.BOTH_LOCATIONS){
                    linearSystem.put("Stop", new TrueTimeTerminator(5000));
                }
                Vector3 wait2Endpoint = wait2Path.getEndpoint();
                if(pickupSecondWobble){
                    //TODO: Develop this further
                    secondWobblePath = new PathBuilder(wait2Path.getEndpoint())
                            .lineTo(31, 15, Angle.degrees(0)).complete();
                    linearSystem.put("Pickup Second Wobble", builder.follow(secondWobblePath), new OrientationTerminator(position, secondWobblePath));
                    wait2Endpoint = secondWobblePath.getEndpoint();
                }



                parkPath = new PathBuilder(wait2Endpoint).lineTo(21, 65, Angle.degrees(0)).complete();

                linearSystem.put("Shutdown Systems", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        hardware.smartDevices.get("SmartCV", SmartCV.class).shutdownTowerTrack();
                        //stateMachine.deactivateState("Intake");
                        stateMachine.deactivateState("Shooter");
                        stateMachine.deactivateState("Load Shooter");
                    }
                }, new TimeTerminator(2));

                linearSystem.put("Park", builder.follow(parkPath), new OrientationTerminator(position, parkPath));

                stateMachine.activateLogic("Update Linear System");
            }
        });

        stateMachine.appendLogicState("Update Linear System", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.update(sensorData, hardwareData);
            }
        });

        eventSystem.onStart("Shooter", new LogicState(stateMachine) {
            final PIDFSystem system = new PIDFSystem(1, 0, 0, 0.1);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //TODO: Put this velocity in SensorData
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                if(turretTarget == TensorTeleop.TARGET.BLUE_GOAL || turretTarget == TensorTeleop.TARGET.RED_GOAL || turretTarget == TensorTeleop.TARGET.NONE){
                    //Targeting the goal
                    hardwareData.setShooter(0.75 + system.getCorrection(4.5 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.316);
                    if(Math.abs(vel - 4.5) < 0.1){
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
                    hardwareData.setShooterLoadArm(0.7);
                    timer = System.currentTimeMillis() + 80; //Wait for indexer to move and shooter to grab ring
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;//Waiting for the "in" move to complete...
                    }
                }
                if(state == 2){
                    hardwareData.setShooterLoadArm(0.925);//Retract the indexer out
                    timer = System.currentTimeMillis() + 80; //Wait for indexer to move and next ring to fall
                    state = 3;
                }
                if(state == 3){
                    if(System.currentTimeMillis() >= timer){
                        if(shoot && shooterReady){//Wait for the next shoot command
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

    enum DELAY_LOCATION{
        FIRST_LOCATION,
        SECOND_LOCATION,
        NO_DELAY,
        BOTH_LOCATIONS;

        private static DELAY_LOCATION[] elements = values();

        public static DELAY_LOCATION fromNumber(int number)
        {
            return elements[number % elements.length];
        }

        public DELAY_LOCATION next()
        {
            return elements[(this.ordinal() + 1) % elements.length];
        }

        public DELAY_LOCATION previous()
        {
            return elements[(this.ordinal() - 1) < 0 ? elements.length - 1 : this.ordinal() - 1];
        }
    }
}
