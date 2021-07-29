package Odometry;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import MathSystems.*;
import State.*;

/**
 * See ConstantVMathUtil.java for details
 * Constant velocity odometer is more accurate then the simple odometer
 * Slightly harder to tune and may be a little less reliable then the simple odometer
 */
public class ConstantVOdometer extends Odometer {
    private Vector3 prevEncoderValues, prevPosition;
    private static final double AUX_ROTATION_CONSTANT = 750.8771; //2305.27659012?
    private double ROT_CONSTANT = 1/(15351.67200); //15178.3496
    private double x, y, rot;
    private long prevTime;
    public ConstantVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity) {
        this(stateMachine, position, velocity, 0, 0);
    }

    public void set(Vector3 position){
        this.x = position.getA() / RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR;
        this.y = -position.getB() / RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR;
        rot = position.getC();
        this.position.set(position);
    }

    @Override
    public void reset() {
        this.x = 0;
        this.y = 0;
        rot = 0;
        position.set(x, y, 0);
        prevPosition.set(x, y, 0);
    }

    public ConstantVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity, double x, double y) {
        super(stateMachine, position, velocity);
        prevEncoderValues = Vector3.ZERO();
        this.x = x / RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR;
        this.y = y / RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR;
        position.set(this.x, this.y, 0);
        rot = 0;
        prevTime = 0;
        prevPosition = Vector3.ZERO();
    }

    @Override
    public void update(SensorData sensors, HardwareData hardwareData) {
        double forInc = ((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0) - prevEncoderValues.getA();
        double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2.0) * ROT_CONSTANT));
        //RobotLog.ii("Encoders", rotInc + " | " + sensors.getOdometryRight() + " | " + sensors.getOdometryLeft() + " | " + prevEncoderValues.getC());
        //double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), sensors.getGyro());
        double strafeInc = (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)) - prevEncoderValues.getB();
        Vector2 pos = MathUtils.toPolar(ConstantVMathUtil.toRobotCentric(forInc, strafeInc, rotInc));
        Vector2 fieldCentric = MathUtils.toCartesian(pos.getA(), pos.getB() - rot);
        x += fieldCentric.getA();
        y += fieldCentric.getB();
        rot = (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2.0) * ROT_CONSTANT);
        double tau = (2 * Math.PI);
        rot = ((rot % tau) + tau) % tau;
        position.set(x * RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR, -y * RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR, rot);
        Vector3 localVel = position.subtract(prevPosition).scale(1.0/MathUtils.nanoToDSec(System.nanoTime() - prevTime));
        prevTime = System.nanoTime();
        if(!Double.isNaN(localVel.getA()) && !Double.isNaN(localVel.getB()) && !Double.isNaN(localVel.getC()) && !Double.isInfinite(localVel.getA()) && !Double.isInfinite(localVel.getB()) && !Double.isInfinite(localVel.getC())) {
            velocity.set(localVel);
        }
        prevPosition.set(position);
        prevEncoderValues.set(((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0), (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)), (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2.0) * ROT_CONSTANT));
    }
}
