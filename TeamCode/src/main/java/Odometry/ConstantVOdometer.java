package Odometry;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import MathUtils.*;
import State.*;

/**
 * See ConstantVMathUtil.java for details
 * Constant velocity odometer is more accurate then the simple odometer
 * Slightly harder to tune and may be a little less reliable then the simple odometer
 */
public class ConstantVOdometer extends Odometer {
    private Vector3 prevEncoderValues, prevPosition;
    private static final double AUX_ROTATION_CONSTANT = 2305.27659012; //2305.27659012?
    private double ROT_CONSTANT = 1.4032E-4;
    private double x, y, rot;
    private long prevTime;
    public ConstantVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity) {
        this(stateMachine, position, velocity, 0, 0);
    }

    public ConstantVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity, double x, double y) {
        super(stateMachine, position, velocity);
        prevEncoderValues = Vector3.ZERO();
        this.x = x / RobotConstants.Skystone.ODOMETRY_TRANSLATION_FACTOR;
        this.y = y / RobotConstants.Skystone.ODOMETRY_TRANSLATION_FACTOR;
        position.set(this.x, this.y, 0);
        rot = 0;
        prevTime = 0;
        prevPosition = Vector3.ZERO();
    }

    @Override
    public void update(SensorData sensors, HardwareData hardwareData) {
        double forInc = ((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0) - prevEncoderValues.getA();
        //double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2) * ROT_CONSTANT));
        double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), sensors.getGyro());
        double strafeInc = (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)) - prevEncoderValues.getB();
        Vector2 pos = MathUtils.toPolar(ConstantVMathUtil.toRobotCentric(forInc, strafeInc, rotInc));
        Vector2 fieldCentric = MathUtils.toCartesian(pos.getA(), pos.getB() + rot);
        x += fieldCentric.getA();
        y += fieldCentric.getB();
        rot = sensors.getGyro();
        double tau = (2 * Math.PI);
        rot = ((rot % tau) + tau) % tau;
        position.set(x * RobotConstants.Skystone.ODOMETRY_TRANSLATION_FACTOR, y * RobotConstants.Skystone.ODOMETRY_TRANSLATION_FACTOR, rot);
        velocity.set(position.subtract(prevPosition).scale(1.0/MathUtils.nanoToSec(System.nanoTime() - prevTime)));
        prevPosition.set(position);
        prevTime = System.nanoTime();
        prevEncoderValues.set(((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0), (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)), sensors.getGyro());
    }
}