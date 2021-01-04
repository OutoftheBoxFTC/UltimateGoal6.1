package Odometry;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.*;
import MathUtils.*;
import State.*;

/**
 * Simple Odometer, uses a basic trig system to calculate position (Straight line assumption)
 * Inaccurate, but requires the least tunings and is the most reliable
 */

public class SimpleOdometer extends Odometer {
    private Vector3 prevEncoderValues, prevPosition;
    private double AUX_ROTATION_CONSTANT = 0;
    private double TRANSLATION_FACTOR = (0.0011);
    private double ROT_CONSTANT = 1.4032E-4;
    private double x, y, rot;
    private long prevTime;

    public SimpleOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity){
        this(stateMachine, position, velocity, 0, 0);
    }

    public SimpleOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity, double x, double y) {
        super(stateMachine, position, velocity);
        prevEncoderValues = Vector3.ZERO();
        prevPosition = Vector3.ZERO();
        this.x = x;
        this.y = y;
        rot = 0;
        prevTime = 0;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardwareData) {
        double forInc = ((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0) - prevEncoderValues.getA();
        //double rotInc = (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2) * ROT_CONSTANT) - prevEncoderValues.getC();
        double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), sensors.getGyro());
        double strafeInc = (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)) - prevEncoderValues.getB();
        double r = Math.sqrt((forInc * forInc) + (strafeInc * strafeInc));
        if(r < 0.05){
            r = 0;
        }
        double theta = rot + Math.atan2(forInc, strafeInc);
        x += r * Math.cos(theta);
        y += r * Math.sin(theta);
        rot = sensors.getGyro();
        double tau = (2 * Math.PI);
        rot = ((rot % tau) + tau) % tau;
        //RobotLog.ii("Odometer", forInc + " " + strafeInc + " " + rotInc + " " + (r * Math.cos(theta)) + " " + (r * Math.sin(theta)));
        position.set(x, y, rot);
        velocity.set(position.subtract(prevPosition).scale(1.0/MathUtils.nanoToSec(System.nanoTime() - prevTime)));
        prevPosition.set(position);
        prevTime = System.nanoTime();
        prevEncoderValues.set(((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0), (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)), sensors.getGyro());
    }
}
