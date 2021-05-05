package Odometry;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import MathSystems.ConstantVMathUtil;
import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;
import State.StateMachine;

/**
 * See ConstantVMathUtil.java for details
 * Constant velocity odometer is more accurate then the simple odometer
 * Slightly harder to tune and may be a little less reliable then the simple odometer
 */
public class AdvancedVOdometer extends Odometer {
    private Vector3 prevEncoderValues, prevPosition;
    private static final double AUX_ROTATION_CONSTANT = 750.8771; //2305.27659012?
    private double ROT_CONSTANT = 1/(15178.3496); //15141.82356
    private double x, y, rot;
    private long prevTime;
    public AdvancedVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity) {
        this(stateMachine, position, velocity, 0, 0);
    }

    public void setKinematicPosition(double x, double y, double r){
        this.position.set(x, y, r);
    }

    public void setKinematicPosition(Vector3 position){
        this.position.set(position);
    }

    @Override
    public void reset() {
        position.set(Vector3.ZERO());
    }

    public AdvancedVOdometer(StateMachine stateMachine, Vector3 position, Vector3 velocity, double x, double y) {
        super(stateMachine, position, velocity);
        prevEncoderValues = Vector3.ZERO();
        position.set(this.x, this.y, 0);
        rot = 0;
        prevTime = 0;
        prevPosition = Vector3.ZERO();
    }

    @Override
    public void update(SensorData sensors, HardwareData hardwareData) {
        double forInc = ((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0) - prevEncoderValues.getA();
        double rotInc = MathUtils.getRadRotDist(prevEncoderValues.getC(), (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2.0) * ROT_CONSTANT));
        double strafeInc = (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)) - prevEncoderValues.getB();
        Vector2 pos = MathUtils.toPolar(ConstantVMathUtil.toRobotCentric(forInc, strafeInc, rotInc));
        Vector2 fieldCentric = MathUtils.toCartesian(pos.getA(), pos.getB() - rot);
        x = fieldCentric.getA();
        y = fieldCentric.getB();
        rot = position.getC();
        rot += rotInc;
        double tau = (2 * Math.PI);
        rot = ((rot % tau) + tau) % tau;
        position.set(position.add(x * RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR, -y * RobotConstants.UltimateGoal.ODOMETRY_TRANSLATION_FACTOR, 0));
        position.setC(rot);
        Vector3 localVel = position.subtract(prevPosition).scale(1.0/MathUtils.nanoToDSec(System.nanoTime() - prevTime));
        prevTime = System.nanoTime();
        if(!Double.isNaN(localVel.getA()) && !Double.isNaN(localVel.getB()) && !Double.isNaN(localVel.getC()) && !Double.isInfinite(localVel.getA()) && !Double.isInfinite(localVel.getB()) && !Double.isInfinite(localVel.getC())) {
            velocity.set(localVel);
        }
        prevPosition.set(position);
        prevEncoderValues.set(((sensors.getOdometryLeft() + sensors.getOdometryRight())/2.0), (sensors.getOdometryAux() - (AUX_ROTATION_CONSTANT * rotInc)), (((sensors.getOdometryRight() - sensors.getOdometryLeft())/2.0) * ROT_CONSTANT));
    }
}
