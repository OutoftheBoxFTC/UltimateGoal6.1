package OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.util.ArrayList;

import Hardware.Hardware;
import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import Hardware.SmartDevices.SmartCV.AprilTagDetectionPipeline;
import Hardware.SmartDevices.SmartCV.SmartCV;
import Hardware.SmartDevices.SmartCV.TowerGoal.CvUtils;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.PIDSystem;
import MathSystems.ProgramClock;
import MathSystems.Vector3;
import Motion.Terminators.TimeTerminator;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.AdvancedVOdometer;
import OpModes.BasicOpmode;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.SingleLogicState;
import State.VelocityDriveState;

@TeleOp
public class OutreachTeleop extends BasicOpmode {
    AdvancedVOdometer odometer, trackingOdo;
    Vector3 position, velocity, timestampedPosition, trackingPos, trackingVel;
    long timestamp = 0, dataTimestamp = 0;
    TARGET turretTarget = TARGET.NONE;
    COLOR color = COLOR.RED;
    boolean shoot = false, shooterReady = false;
    double speedMod = 1, angOffset = 0, pitchOffset = 0;
    public static double ARM_IDLE = 1400, ARM_DOWN = RobotConstants.UltimateGoal.INTAKE_BLOCKER_DOWN;
    boolean notInRange = false;

    AprilTagDetectionPipeline pipeline;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    public OutreachTeleop() {
        super(new UltimateGoalHardware());
    }

    @Override
    public void setup() {
        pipeline = new AprilTagDetectionPipeline();
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
                if(gamepad1.back){
                    color = COLOR.BLUE;
                }
                if(gamepad1.start){
                    color = COLOR.RED;
                }
                if(color == COLOR.RED){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                }else if (color == COLOR.BLUE){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
                }
                hardware.smartDevices.get("SmartCV", SmartCV.class).disableRingTrack();
                hardware.smartDevices.get("SmartCV", SmartCV.class).setPitchOffset(29.980765);//29.980765 ; 27.17
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
                final OpenCvWebcam tower =hardware.smartDevices.get("SmartCV", SmartCV.class).getTower();
                //hardware.smartDevices.get("SmartCV", SmartCV.class).getTower().setPipeline(pipeline);
                tower.setPipeline(pipeline);
                tower.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        tower.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
                        FtcDashboard.getInstance().startCameraStream(tower, 30);
                        RobotLog.i("********OPENED*********");
                    }

                    @Override
                    public void onError(int errorCode) {

                    }
                });
            }
        });

        eventSystem.onStart("Wobble Release", new LogicState(stateMachine) {
            long timer = 0;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                timer = System.currentTimeMillis() + 3000;
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_IN);
                hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_IN);
                hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
                hardwareData.setWobbleOneuseLeft(RobotConstants.UltimateGoal.ONEUSE_LEFT_ARM_RELEASE);
                if(System.currentTimeMillis() > timer){
                    deactivateThis();
                }
            }
        });

        eventSystem.onStart("Monitoring", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setWobbleForkLeft(RobotConstants.UltimateGoal.WOBBLE_FORK_LEFT_IN);
                hardwareData.setWobbleForkRight(RobotConstants.UltimateGoal.WOBBLE_FORK_RIGHT_IN);
                hardwareData.setWobbleFourbarRight(RobotConstants.UltimateGoal.WOBBLE_ARM_RIGHT_DOWN);
                hardwareData.setWobbleFourbarLeft(RobotConstants.UltimateGoal.WOBBLE_ARM_LEFT_DOWN);
                hardwareData.setWobbleOneuseRight(RobotConstants.UltimateGoal.ONEUSE_RIGHT_ARM_RELEASE);
                hardwareData.setWobbleOneuseLeft(RobotConstants.UltimateGoal.ONEUSE_LEFT_ARM_RELEASE);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Pitch And Yaw Offsets", angOffset + " " + pitchOffset);
                telemetry.addData("Target", turretTarget);
                telemetry.addData("Color", color);
                telemetry.addData("Timestamped Pos", timestampedPosition);
                telemetry.addData("Pos", trackingPos);
                telemetry.addData("FPS", fps);

            }
        });

        eventSystem.onStart("Temps", new LogicState(stateMachine) {
            volatile float temp = 0;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        while(opModeIsActive()) {
                            Process process;
                            try {
                                process = Runtime.getRuntime().exec("cat sys/class/thermal/thermal_zone0/temp");
                                process.waitFor();
                                BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
                                String line = reader.readLine();
                                if (line != null) {
                                    //float temp = Float.parseFloat(line);
                                    temp = Float.parseFloat(line) / 1000.0f;
                                } else {
                                    temp = -1;
                                }
                            } catch (Exception e) {
                                temp = -1;
                            }
                        }
                    }
                }).start();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Temp", temp);
            }
        });

        eventSystem.onStart("Freq", new LogicState(stateMachine) {
            float temp;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        while(opModeIsActive()) {
                            Process process;
                            try {
                                process = Runtime.getRuntime().exec("cat sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
                                process.waitFor();
                                BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
                                String line = reader.readLine();
                                if (line != null) {
                                    temp = Float.parseFloat(line);
                                    //telemetry.addData("Freq", temp);
                                } else {
                                    //telemetry.addData("Freq", "N/A");
                                    temp = -1;
                                }
                            } catch (Exception e) {
                                //telemetry.addData("Freq", e.getLocalizedMessage());
                                temp = -1;
                            }
                        }
                    }
                });
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Freq", temp);
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
                    if(isStarted()) {
                        stateMachine.activateLogic("Change Blinkin Lights");
                    }
                }
                if((gamepad1.start || gamepad1.back || notInRange) && !stateMachine.logicStateActive("Change Blinkin Lights")){
                    hardwareData.setPattern(gamepad1.start ? RevBlinkinLedDriver.BlinkinPattern.RED :
                            gamepad1.back ? RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                }else if(!stateMachine.logicStateActive("Change Blinkin Lights")){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
                }
            }
        });

        stateMachine.appendLogicState("Change Blinkin Lights", new LogicState(stateMachine) {
            long timer = 0;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(timer == 0){
                    timer = System.currentTimeMillis() + 400;
                }
                if(gamepad1.start || gamepad1.back){
                    hardwareData.setPattern(gamepad1.start ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    return;
                }
                if(notInRange){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
                    return;
                }
                hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                if(timer < System.currentTimeMillis()){
                    hardwareData.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);
                    timer = 0;
                    stateMachine.deactivateState("Change Blinkin Lights");
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
                ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

                // If there's been a new frame...
                if(detections != null)
                {

                    // If we don't see any tags
                    if(detections.size() == 0)
                    {
                        numFramesWithoutDetection++;

                        // If we haven't seen a tag for a few frames, lower the decimation
                        // so we can hopefully pick one up if we're e.g. far back
                        if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                        {
                            pipeline.setDecimation(DECIMATION_LOW);
                        }
                    }
                    // We do see tags!
                    else
                    {
                        numFramesWithoutDetection = 0;

                        // If the target is within 1 meter, turn on high decimation to
                        // increase the frame rate
                        if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                        {
                            pipeline.setDecimation(DECIMATION_HIGH);
                        }

                        for(AprilTagDetection detection : detections)
                        {
                            angle = Math.toDegrees(Math.atan2((640 - detection.center.x) - 640/2.0, CvUtils.calcPinholeHor(70, 640, 480)));
                        }
                    }

                    telemetry.update();
                }

                hardwareData.setTurret(UGUtils.getTurretValue(angle + angOffset));
            }
        });

        eventSystem.onStart("Drive", new VelocityDriveState(stateMachine) {
            final double minSpeed = 0.2;
            final double minDist = 8;
            final double maxSpeed = 1;
            final double maxDist = 35;

            double speed = 1;
            double lastDist = -1;

            final Vector3 lastPos = Vector3.ZERO();
            final PIDSystem x = new PIDSystem(0.1, 0.4, 0.2, 1);
            final PIDSystem y = new PIDSystem(0.1, 0.4, 0.2, 1);
            final PIDSystem r = new PIDSystem(0.1, 0.1, 0.2, 1);

            @Override
            public Vector3 getVelocities() {
                hardware.smartDevices.get("Front Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Front Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Left", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.smartDevices.get("Back Right", SmartMotor.class).getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Poses", trackingPos + " " + lastPos);
                if(Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
                    return new Vector3(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x).scale(speed).scale(speedMod);
                }
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double slope = (maxSpeed - minSpeed) / (maxDist - minDist);
                double intercept = maxSpeed - (slope * maxDist);

                hardware.disableDevice(Hardware.HardwareDevices.DISTANCE_SENSOR);
                speed = 1;
                lastDist = -1;
                telemetry.addData("Distance", sensorData.getDistance());
            }
        });

        eventSystem.onStart("Driver", new LogicState(stateMachine) {
            boolean lastUp = false, lastDown = false, prevShoot = false;
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
                if(!gamepad1.a) {
                    if (gamepad1.x) {
                        if (color == COLOR.BLUE) {
                            turretTarget = TARGET.BLUE_POWERSHOT_LEFT;
                        } else {
                            turretTarget = TARGET.RED_POWERSHOT_LEFT;
                        }
                    } else if (gamepad1.y) {
                        if (color == COLOR.BLUE) {
                            turretTarget = TARGET.BLUE_POWERSHOT_CENTER;
                        } else {
                            turretTarget = TARGET.RED_POWERSHOT_CENTER;
                        }
                    } else if (gamepad1.b) {
                        if (color == COLOR.BLUE) {
                            turretTarget = TARGET.BLUE_POWERSHOT_RIGHT;
                        } else {
                            turretTarget = TARGET.RED_POWERSHOT_RIGHT;
                        }
                    } else {
                        if (color == COLOR.BLUE) {
                            turretTarget = TARGET.BLUE_GOAL;
                        } else {
                            turretTarget = TARGET.RED_GOAL;
                        }
                    }
                }
                //Shooting is not on a hair trigger because it has gotten "stuck" before, constantly returning ~0.1
                if(gamepad1.left_trigger > 0.15){
                    if(gamepad1.x || gamepad1.y || gamepad1.b){
                        if(!prevShoot){
                            shoot = true;
                        }
                    }else {
                        shoot = true;
                    }
                }
                prevShoot = gamepad1.left_trigger > 0.15;
                if(gamepad1.right_trigger > 0.15){
                    speedMod = 0.5;
                }else{
                    speedMod = 1;
                }

                if(gamepad1.dpad_up && !lastUp){
                    state ++;
                    if(state == 4){
                        state = 0;
                    }
                }
                if(gamepad1.dpad_down && !lastDown){
                    state --;
                    if(state < 0){
                        state = 3;
                    }
                }
                /**
                if(gamepad1.a && !stateMachine.logicStateActive("Auto Powershots")){
                    stateMachine.activateLogic("Auto Powershots");
                }
                if(!gamepad1.a && stateMachine.logicStateActive("Auto Powershots")){
                    stateMachine.deactivateState("Auto Powershots");
                }
                */
                if(gamepad1.a){
                    turretTarget = TARGET.FORWARD;
                }

                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;
            }
        });

        stateMachine.appendLogicState("Auto Powershots", new LogicState(stateMachine) {
            boolean inited = false;
            LinearEventSystem linearSystem;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(!inited){
                    stateMachine.appendLogicState("Stop", new LogicState(stateMachine) {
                        @Override
                        public void update(SensorData sensorData, HardwareData hardwareData) {

                        }
                    });
                    linearSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.STOP_ALL);
                    linearSystem.put("Stop", new TrueTimeTerminator(800));
                    linearSystem.put("Shoot Left", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = color == COLOR.BLUE ? TARGET.BLUE_POWERSHOT_LEFT : TARGET.RED_POWERSHOT_RIGHT;
                            shoot = true;
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));

                    linearSystem.put("Aim Centre", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = color == COLOR.BLUE ? TARGET.BLUE_POWERSHOT_CENTER : TARGET.RED_POWERSHOT_CENTER;
                        }
                    }, new TrueTimeTerminator(300));

                    linearSystem.put("Shoot Centre", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = color == COLOR.BLUE ? TARGET.BLUE_POWERSHOT_RIGHT : TARGET.RED_POWERSHOT_LEFT;
                            shoot = true;
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));

                    linearSystem.put("Aim Right", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = OutreachTeleop.TARGET.BLUE_POWERSHOT_RIGHT;
                        }
                    }, new TrueTimeTerminator(500));

                    linearSystem.put("Shoot Right", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            turretTarget = OutreachTeleop.TARGET.BLUE_POWERSHOT_RIGHT;
                            shoot = true;
                            stateMachine.activateLogic("Vision Update");
                        }
                    }, new TimeTerminator(7));

                    linearSystem.put("Stop", new TrueTimeTerminator(500));
                    linearSystem.put("Deactive This System", new SingleLogicState(stateMachine) {
                        @Override
                        public void main(SensorData sensorData, HardwareData hardwareData) {
                            stateMachine.deactivateState("Auto Powershots");
                        }
                    }, new TimeTerminator(7));
                    inited = true;
                }
                linearSystem.update(sensorData, hardwareData);
            }

            @Override
            public void onStop(SensorData sensorData, HardwareData hardwareData) {
                linearSystem.reset();
            }
        });

        eventSystem.onStart("Operator", new LogicState(stateMachine) {
            boolean dpadUp, dpadDown, dpadLeft, dpadRight;
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

                if(gamepad2.dpad_up && !dpadUp){
                    pitchOffset -= 0.01;
                }
                if(gamepad2.dpad_down && !dpadDown){
                    pitchOffset += 0.01;
                }
                if(gamepad2.dpad_right && !dpadRight){
                    angOffset -= 1;
                }
                if(gamepad2.dpad_left && !dpadLeft){
                    angOffset += 1;
                }

                if(gamepad2.right_trigger > 0.15){
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(RobotConstants.UltimateGoal.INTAKE_BLOCKER_UP)+0.1);
                }else{
                    hardwareData.setIntakeShield(UGUtils.PWM_TO_SERVO(RobotConstants.UltimateGoal.INTAKE_BLOCKER_UP));
                }

                dpadUp = gamepad2.dpad_up;
                dpadDown = gamepad2.dpad_down;
                dpadLeft = gamepad2.dpad_left;
                dpadRight = gamepad2.dpad_right;
            }
        });

        eventSystem.onStart("Shooter", new LogicState(stateMachine) {
            final PIDFSystem system = new PIDFSystem(1, 0, 0, 0.1);
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                //TODO: Put this velocity in SensorData
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                if(Math.abs(gamepad1.right_stick_y) > 0.9){
                    if(Math.abs(vel) > 2) {
                        hardwareData.setShooter(0 + system.getCorrection(0 - vel, 0));
                    }else{
                        hardwareData.setShooter(0);
                    }
                    //hardwareData.setShooterTilt(0.41 + pitchOffset);
                    shooterReady = false;
                    return;
                }
                hardwareData.setShooterTilt(0.355 + pitchOffset);
                if(turretTarget == TARGET.BLUE_GOAL || turretTarget == TARGET.RED_GOAL || turretTarget == TARGET.NONE){
                    //Targeting the goal
                    hardwareData.setShooter(0.75 + system.getCorrection(4 - vel, shoot ? 1 : 0));
                    if(gamepad1.right_bumper || gamepad1.left_bumper){
                       // hardwareData.setShooterTilt(0.355 + pitchOffset);
                    }else {
                        //hardwareData.setShooterTilt(0.355 + pitchOffset);
                    }
                    if(Math.abs(vel - 4) < 0.75){
                        shooterReady = true;
                    }else{
                        shooterReady = false;
                    }
                }else {
                    //Targeting the powershots
                    hardwareData.setShooter(0.7 + system.getCorrection(3.9 - vel, shoot ? 1 : 0));
                    //hardwareData.setShooterTilt(0.365 + pitchOffset);
                    if(Math.abs(vel - 3.9) < 0.15){
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
                    timer = System.currentTimeMillis() + 100; //Wait for indexer to move and shooter to grab ring
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
        FORWARD,
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
