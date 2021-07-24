package OpModes.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Hardware.Hardware;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.Vector3;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.Terminator;
import Motion.Terminators.TimeTerminator;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.AdvancedVOdometer;
import Odometry.ConstantVOdometer;
import OpModes.BasicOpmode;
import OpModes.TeleOp.TensorTeleop;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@Autonomous
public class BlueInsideAuto extends BasicOpmode {
    long START_DELAY_TIME_MS = 1000;
    long DELAY_TIME_MS = 1000;

    ConstantVOdometer odometer;
    AdvancedVOdometer targetingOdometer;
    Vector3 position, velocity, targetingPos, targetingVel;
    boolean doStarterStack = false, pickupSecondWobble = false, doPowerShots = true;
    DELAY_LOCATION delayLocation = DELAY_LOCATION.NO_DELAY;
    int startingStack = 0;
    LinearEventSystem linearSystem;
    CrosstrackBuilder builder;
    boolean shooterReady = false, shoot = false;
    TensorTeleop.TARGET turretTarget = TensorTeleop.TARGET.BLUE_GOAL;
    private long timestamp;
    private Vector3 timestampedPosition = Vector3.ZERO();
    private long dataTimestamp;

    public BlueInsideAuto() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        hardware.disableDevice(Hardware.HardwareDevices.DISTANCE_SENSOR);
        hardware.deRegisterDevice(Hardware.HardwareDevices.DISTANCE_SENSOR);

        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        targetingPos = Vector3.ZERO();
        targetingVel = Vector3.ZERO();
        targetingOdometer = new AdvancedVOdometer(stateMachine, targetingPos, targetingVel);
        odometer = new ConstantVOdometer(stateMachine, position, velocity);

        linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);

        builder = new CrosstrackBuilder(stateMachine, position);

        eventSystem.onStart("Odometer", odometer);

        eventSystem.onInit("Targeting Odometer", targetingOdometer);

        eventSystem.onInit("Monitoring", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("States", stateMachine.getActiveStates());
                telemetry.addData("Targeting Pos", targetingPos);
            }
        });

        eventSystem.onInit("Vision Update", new LogicState(stateMachine) {
            SmartCV smartCV;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                smartCV = hardware.getSmartDevices().get("SmartCV", SmartCV.class);
                smartCV.setPitchOffset(29.695);
                smartCV.setOuter(false);
                smartCV.setBlue(true);
                smartCV.setVelocity(velocity);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //This timestamp is updated BEFORE the AI runs, so we grab our position and save it
                if(smartCV.getTimestamp() != timestamp){
                    timestamp = smartCV.getTimestamp();
                    timestampedPosition.set(targetingPos);
                }
                //This timestamp is updated AFTER the AI runs
                if(smartCV.getDataTimestamp() != dataTimestamp){
                    dataTimestamp = smartCV.getDataTimestamp();
                    //Calculate how much we have moved since the AI started running
                    Vector3 posdiff = targetingPos.subtract(timestampedPosition);
                    //Retroactively set and update our position, by adding the delta to our AI position
                    //Because the AI position will be old (~650ms behind)
                    Vector3 newpos = new Vector3(smartCV.getPosition()[0], smartCV.getPosition()[1], 0);
                    targetingOdometer.setKinematicPosition(newpos.add(posdiff));
                }
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
                        idx = 4;
                    }
                }
                if(gamepad1.dpad_down && !pressedDown){
                    idx ++;
                    if(idx > 4){
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
                    telemetry.addData("Do Power Shots", "<" + doPowerShots + ">");
                    if((gamepad1.dpad_left && !pressedLeft) || (gamepad1.dpad_right && !pressedRight)){
                        doPowerShots = !doPowerShots;
                    }
                }else{
                    telemetry.addData("Do Power Shots", doPowerShots);
                }

                pressedDown = gamepad1.dpad_down;
                pressedUp = gamepad1.dpad_up;
                pressedLeft = gamepad1.dpad_left;
                pressedRight = gamepad1.dpad_right;
            }
        });

        stateMachine.appendLogicState("Turret", new LogicState(stateMachine) {
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
                        deltaX = -(targetingPos.getA()+35);
                        deltaY = -(targetingPos.getB());
                        break;
                    case RED_GOAL:
                        deltaX = -(targetingPos.getA()-35);
                        deltaY = -(targetingPos.getB());
                        break;
                    case RED_POWERSHOT_LEFT:
                        deltaX = -(targetingPos.getA() - (35 - goalOffset - powershotOffset - powershotOffset));
                        deltaY = -(targetingPos.getB());
                        break;
                    case RED_POWERSHOT_CENTER:
                        deltaX = -(targetingPos.getA() - (35 - goalOffset - powershotOffset));
                        deltaY = -(targetingPos.getB());
                        break;
                    case RED_POWERSHOT_RIGHT:
                        deltaX = -(targetingPos.getA() - (35 - goalOffset));
                        deltaY = -(targetingPos.getB());
                        break;
                    case BLUE_POWERSHOT_LEFT:
                        deltaX = -(targetingPos.getA() + (35 - goalOffset - 2));
                        deltaY = -(targetingPos.getB());
                        break;
                    case BLUE_POWERSHOT_CENTER:
                        deltaX = -(targetingPos.getA() + (35 - goalOffset - powershotOffset));
                        deltaY = -(targetingPos.getB());
                        break;
                    case BLUE_POWERSHOT_RIGHT:
                        deltaX = -(targetingPos.getA() + (35 - goalOffset - powershotOffset - powershotOffset));
                        deltaY = -(targetingPos.getB());
                        break;
                    case NONE:
                        break;
                }
                if(velocity.getVector2().length() > 4){
                    double dist = hardware.smartDevices.get("SmartCV", SmartCV.class).getRange();
                    //deltaX -= velocity.getA() * (dist / 120);
                    //deltaY -= velocity.getB() * (dist / 120);
                }
                hardwareData.setTurret(UGUtils.getTurretValue(Math.toDegrees(MathUtils.getRadRotDist(targetingPos.getC(), -Math.atan2(deltaX, deltaY)))));
            }
        });

        eventSystem.onInit("Vision", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                startingStack = (int) sensorData.getRings();
                if(startingStack == 0){
                    doStarterStack = false;
                }
                hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(RobotConstants.UltimateGoal.INTAKE_BLOCKER_UP));
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

        eventSystem.onInit("Init Servos", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_IN);
                hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_IN);

                hardwareData.setTurret(UGUtils.getTurretValue(20));
            }
        });

        stateMachine.appendLogicState("Turret Activation", new SingleLogicState(stateMachine) {
            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.activateLogic("Turret");
            }
        });

        eventSystem.onStart("Setup Systems", new SingleLogicState(stateMachine) {
            Path highgoalPath, startingStackPath, wait1Path, wobblePath, wait2Path, secondWobblePath, shootSecondPath, parkPath;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                highgoalPath = new PathBuilder(0, 0, Angle.degrees(0)).lineTo(2, 50).complete();
                startingStackPath = new PathBuilder(highgoalPath.getEndpoint()).lineTo(-3, 35).lineTo(-3, 55).complete();
                turretTarget = TensorTeleop.TARGET.BLUE_GOAL;
                hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(RobotConstants.UltimateGoal.INTAKE_BLOCKER_UP));
            }

            @Override
            public void main(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.put("Stop", new TrueTimeTerminator(START_DELAY_TIME_MS));

                linearSystem.put("Release Mechs", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        //hardwareData.setIntakePower(1);
                        //hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(RobotConstants.UltimateGoal.INTAKE_BLOCKER_DOWN));
                        turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_LEFT;
                        stateMachine.activateLogic("Turret");
                    }
                }, new TimeTerminator(15));

                linearSystem.put("Highgoal Shoot Pos", builder.follow(highgoalPath), new OrientationTerminator(position, highgoalPath, 10));

                linearSystem.put("Stop For Shoot", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        return Vector3.ZERO();
                    }

                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {

                    }
                }, new TimeTerminator(15));

                linearSystem.put("Stop", new TrueTimeTerminator(1200));

                linearSystem.put("Stop", new Terminator() {
                    @Override
                    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
                        return hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack()
                                && (System.currentTimeMillis() - hardware.smartDevices.get("SmartCV", SmartCV.class).getDataTimestamp()) < 1500;
                    }
                });

                if(doPowerShots) {

                    linearSystem.put("Shoot Left", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_LEFT;
                            shoot = true;
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));

                    linearSystem.put("Aim Centre", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_CENTER;
                        }
                    }, new TrueTimeTerminator(500));

                    linearSystem.put("Shoot Centre", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_CENTER;
                            shoot = true;
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));

                    linearSystem.put("Aim Right", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_RIGHT;
                        }
                    }, new TrueTimeTerminator(500));

                    linearSystem.put("Shoot Right", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_POWERSHOT_RIGHT;
                            shoot = true;
                            stateMachine.activateLogic("Vision Update");
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));
                }else{
                    linearSystem.put("Aim Highgoal", new LogicState(stateMachine) {
                        @Override
                        public void update(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = TensorTeleop.TARGET.BLUE_GOAL;
                        }
                    }, new TrueTimeTerminator(500));
                    linearSystem.put("Shoot Rings", new LogicState(stateMachine) {
                        @Override
                        public void update(SensorData sensorData, HardwareData hardwareData) {
                            shoot = true;
                        }
                    }, new TrueTimeTerminator(1000));
                }

                PathBuilder wobbleBuilder = new PathBuilder(highgoalPath.getEndpoint());
                if(startingStack == 0){
                    wobblePath = wobbleBuilder.lineTo(-24, 84).complete();
                }else if(startingStack == 1){
                    wobblePath = wobbleBuilder.lineTo(0, 105).complete();
                }else{
                    wobblePath = wobbleBuilder.lineTo(-27, 119, Angle.degrees(-45)).complete();
                }
                linearSystem.put("Wobble Path", builder.follow(wobblePath), new OrientationTerminator(position, wobblePath));

                linearSystem.put("Deploy Oneuse", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        stateMachine.activateLogic("Turret");
                        //hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_CHANGE);
                        //hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_CHANGE);
                        hardwareData.setWobbleOneuseLeft(RobotConstants.UltimateGoal.ONEUSE_LEFT_ARM_RELEASE);
                    }
                }, new TrueTimeTerminator(1000));

                linearSystem.put("Fourbar Back", new SingleLogicState(stateMachine) {
                    @Override
                    public void main(SensorData sensorData, HardwareData hardwareData) {
                        hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                        hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                    }
                }, new TrueTimeTerminator(500));

                linearSystem.put("Intake On", new SingleLogicState(stateMachine) {
                    @Override
                    public void main(SensorData sensorData, HardwareData hardwareData) {
                        stateMachine.appendLogicState("IntakeSys", new LogicState(stateMachine) {
                            @Override
                            public void update(SensorData sensorData, HardwareData hardwareData) {
                                hardwareData.setIntakePower(1);
                            }
                        });
                        stateMachine.activateLogic("IntakeSys");
                    }
                }, new TimeTerminator(7));

                wait2Path = new PathBuilder(wobblePath.getEndpoint()).lineTo(-8, 120, Angle.degrees(0)).lineTo(21, 122, Angle.degrees(-83)).complete();
                linearSystem.put("Wait 2 Path", builder.follow(wait2Path, 2, 0.6, 0.15), new OrientationTerminator(position, wait2Path));

                if(delayLocation == DELAY_LOCATION.SECOND_LOCATION || delayLocation == DELAY_LOCATION.BOTH_LOCATIONS){
                    linearSystem.put("Stop", new TrueTimeTerminator(5000));
                }
                Vector3 wait2Endpoint = wait2Path.getEndpoint();

                shootSecondPath = new PathBuilder(wait2Endpoint)
                        .lineTo(21, 100)
                        .lineTo(3, 50, Angle.degrees(0)).complete();

                linearSystem.put("Turret Activation", new TrueTimeTerminator(500));

                linearSystem.put("Second Shot Drive", builder.follow(shootSecondPath), new OrientationTerminator(position, shootSecondPath, 10));

                linearSystem.put("Stop For Shoot 2", new VelocityDriveState(stateMachine) {
                    @Override
                    public Vector3 getVelocities() {
                        turretTarget = TensorTeleop.TARGET.BLUE_GOAL;
                        return Vector3.ZERO();
                    }

                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {

                    }
                }, new Terminator() {
                    @Override
                    public boolean shouldTerminate(SensorData sensorData, HardwareData hardwareData) {
                        return hardware.smartDevices.get("SmartCV", SmartCV.class).getTrack()
                                && (System.currentTimeMillis() - hardware.smartDevices.get("SmartCV", SmartCV.class).getDataTimestamp()) < 1500;
                    }
                });

                linearSystem.put("Stop", new TrueTimeTerminator(500));

                linearSystem.put("Shoot Highgoal", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        shoot = true;
                        turretTarget = TensorTeleop.TARGET.BLUE_GOAL;
                    }
                }, new TrueTimeTerminator(1000));

                parkPath = new PathBuilder(shootSecondPath.getEndpoint()).lineTo(3, 75, Angle.degrees(0)).complete();

                linearSystem.put("Shutdown Systems", new LogicState(stateMachine) {
                    @Override
                    public void update(SensorData sensorData, HardwareData hardwareData) {
                        //hardware.smartDevices.get("SmartCV", SmartCV.class).shutdownTowerTrack();
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
                    hardwareData.setShooter(0.75 + system.getCorrection(4 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.355);
                    if(Math.abs(vel - 4) < 0.15){
                        shooterReady = true;
                    }else{
                        shooterReady = false;
                    }
                }else {
                    //Targeting the powershots
                    hardwareData.setShooter(0.7 + system.getCorrection(3.9 - vel, shoot ? 1 : 0));
                    hardwareData.setShooterTilt(0.365);
                    if(Math.abs(vel - 3.9) < 0.25){
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
                    timer = System.currentTimeMillis() + 110; //Wait for indexer to move and shooter to grab ring
                    state = 1;
                }
                if(state == 1){
                    if(System.currentTimeMillis() >= timer){
                        state = 2;//Waiting for the "in" move to complete...
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
