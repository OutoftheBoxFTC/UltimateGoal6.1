package OpModes.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

import Hardware.*;
import MathUtils.*;
import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import MathUtils.Vector3;
import Odometry.ConstantVOdometer;
import Odometry.Odometer;
import OpModes.BasicOpmode;
import State.GamepadDriveState;
import State.LogicState;
@TeleOp
public class OdometryTuner extends BasicOpmode {
    Vector3 position, velocity;
    Odometer odometer;
    public OdometryTuner() {
        super(new SkystoneHardware());
    }

    @Override
    public void setup() {
        hardware.registerAll();
        hardware.enableAll();
        position = Vector3.ZERO();
        velocity = Vector3.ZERO();
        odometer = new ConstantVOdometer(stateMachine, position, velocity);
        eventSystem.onStart("Drive", new GamepadDriveState(stateMachine, gamepad1));
        eventSystem.onStart("Odometer", odometer);
        eventSystem.onStart("Tunings", new LogicState(stateMachine) {
            double prevAux = 0, prevRot = 0;
            ArrayList<Double> vals = new ArrayList<>();
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                double aux = sensorData.getOdometryAux() - prevAux;
                double rot = MathUtils.getRadRotDist(sensorData.getGyro(), prevRot);
                double factor = 0;
                factor = (((sensorData.getOdometryRight() - sensorData.getOdometryLeft())/2.0))/(2 * Math.PI * 30);
                if(rot != 0) {
                    if (Math.abs(factor) > 500) {
                        vals.add(factor);
                    }
                }
                telemetry.addData("Factor", factor);
                prevAux = ((sensorData.getOdometryRight() - sensorData.getOdometryLeft())/2.0);
                prevRot = sensorData.getGyro();
            }
        });
        eventSystem.onStart("Telemetry", new LogicState(stateMachine) {
            @Override
            public void update(SensorData sensorData, HardwareData hardwareData) {
                telemetry.addData("Position", position);
                telemetry.addData("FPS", fps);
                telemetry.addData("Pods", new Vector3(sensorData.getOdometryLeft(), sensorData.getOdometryRight(), sensorData.getOdometryAux()));
            }
        });
    }
}
