package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.text.DecimalFormat;

import Hardware.HarwareUtils.UGUtils;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.SmartDevices.SmartCV.TowerCV;
import Hardware.SmartDevices.SmartMotor.SmartMotor;
import Hardware.UltimateGoalHardware;
import MathSystems.Angle;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import Motion.CrosstrackDrive.CrosstrackBuilder;
import Motion.DriveToPoint.DriveToPointBuilder;
import Motion.Path.Path;
import Motion.Path.PathBuilder;
import Motion.Terminators.OrientationTerminator;
import Motion.Terminators.TimeTerminator;
import Motion.Terminators.TrueTimeTerminator;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.EventSystem.LinearEventSystem;
import State.LogicState;
import State.VelocityDriveState;

@TeleOp
public class AutoTurretTest extends BasicOpmode {
    Odometer odometer;
    Vector3 position, velocity;
    double[] powershots;
    public AutoTurretTest() {
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

        eventSystem.onInit("Logging", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position);
                telemetry.addData("Active States", stateMachine.getActiveStates());

                double[] powershots = hardware.getSmartDevices().get("TowerCam", TowerCV.class).getPowershots();
                DecimalFormat format = new DecimalFormat("#.##");

                telemetry.addData("Powershots", format.format(powershots[0]) + " " + format.format(powershots[1]) + " " + format.format(powershots[2]));
            }
        });

        eventSystem.onStart("SpinShooter", new LogicState(stateMachine) {
            PIDSystem system = new PIDSystem(0.8, 0, 0);
            double target = 4.75, tilt = 0.37;
            boolean lastLeft = false, lastRight = false, lastdown = false, lastup = false;

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double vel = hardware.getSmartDevices().get("Shooter Right", SmartMotor.class).getVelocity();
                hardwareData.setShooter(0.7 + system.getCorrection(target - vel)); //0.75 | 4.5
                hardwareData.setShooterTilt(tilt);
                telemetry.addData("Vel", target);
            }
        });

        eventSystem.onStart("Load Shooter", new LogicState(stateMachine) {
            int state = 0;
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
                        state = 0;
                        deactivateThis();
                    }
                }
                telemetry.addData("state", state);
            }
        });

        stateMachine.appendDriveState("Stop", new VelocityDriveState(stateMachine) {
            @Override
            public Vector3 getVelocities() {
                return Vector3.ZERO();
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                if(!stateMachine.logicStateActive("Turret") && hardware.getSmartDevices().get("TowerCam", TowerCV.class).getTrack()){
                    stateMachine.activateLogic("Turret");
                }
            }
        });

        stateMachine.appendLogicState("End", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {

            }
        });

        stateMachine.appendLogicState("Turret", new LogicState(stateMachine) {
            LinearEventSystem eventSystem;

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                eventSystem = new LinearEventSystem(stateMachine, LinearEventSystem.ENDTYPE.CONTINUE_LAST);
                eventSystem.put("End", new TrueTimeTerminator(500));
                eventSystem.put("Shoot First", new TrueTimeTerminator(500));
                eventSystem.put("Load Shooter", new TrueTimeTerminator(550));
                eventSystem.put("Shoot Second", new TrueTimeTerminator(500));
                eventSystem.put("Load Shooter", new TrueTimeTerminator(400));
                eventSystem.put("Shoot Third", new TrueTimeTerminator(500));
                eventSystem.put("Load Shooter", new TrueTimeTerminator(400));
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                eventSystem.update(sensorData, hardwareData);
            }
        });

        eventSystem.onInit("Get Pos", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                powershots = hardware.getSmartDevices().get("TowerCam", TowerCV.class).getPowershots();
                //powershots = new double[]{2, 8, 13};
            }
        });

        eventSystem.onInit("Shoot First", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {

            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[0]));
            }
        });

        stateMachine.appendLogicState("Shoot Second", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[1]));
            }
        });

        stateMachine.appendLogicState("Shoot Third", new LogicState(stateMachine) {

            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                hardwareData.setTurret(UGUtils.getTurretValue(powershots[2]));
            }
        });

        eventSystem.onStart("Drive", new LogicState(stateMachine) {
            OrientationTerminator orientationTerminator;
            @Override
            public void init(SensorData sensorData, HardwareData hardwareData) {
                Path path = new PathBuilder(0, 0, Angle.degrees(0))
                        .lineTo(-5, 50)
                        .complete();
                CrosstrackBuilder builder = new CrosstrackBuilder(stateMachine, position);
                stateMachine.appendDriveState("DriveState", builder.follow(path, 0, 0.6, 0.15));
                //stateMachine.appendDriveState("DriveState", new DriveToPointBuilder(stateMachine, position).setSpeed(0.4).setRotPrec(5).setTarget(path.getEndpoint().getVector2()).setMinimums(0.15).complete());
                orientationTerminator = new OrientationTerminator(position, path.getEndpoint(), 2, 2);
            }

            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                stateMachine.setActiveDriveState("DriveState");
                if(orientationTerminator.shouldTerminate(sensorData, hardwareData)){
                    stateMachine.setActiveDriveState("Stop");
                    deactivateThis();
                }
            }
        });
    }
}
