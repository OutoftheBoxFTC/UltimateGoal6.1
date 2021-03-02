package Motion.PIDDriveToPoint;

import com.qualcomm.robotcore.util.RobotLog;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import MathSystems.MathUtils;
import MathSystems.PIDFSystem;
import MathSystems.PIDSystem;
import MathSystems.Vector3;
import MathSystems.Vector4;
import State.StateMachine;
import State.VelocityDriveState;

/**
 * Drives to a given point using a straight line method
 * the setTarget() function sets the target to drive to
 */

public abstract class PIDDriveToPoint extends VelocityDriveState {
    public Vector3 position, localTarget, startTarget;
    private Vector3 target, velocity, robotVelocity;
    private double power = 0, slowdownRadius, slowdownAngle, minimums;
    private PIDFSystem xSystem, ySystem, rSystem;
    public PIDDriveToPoint(StateMachine stateMachine, Vector3 position, Vector3 robotVelocity, Vector3 target, double power, double slowdownRadius, double slowdownAngle, double minimums, Vector4 driveGain, Vector4 rotGain) {
        super(stateMachine);
        this.position = position;
        this.target = target;
        localTarget = Vector3.ZERO();
        this.power = power;
        this.velocity = Vector3.ZERO();
        this.slowdownRadius = slowdownRadius;
        this.slowdownAngle = slowdownAngle;
        this.xSystem = new PIDFSystem(driveGain.getA(), driveGain.getB(), driveGain.getC(), driveGain.getD());
        this.ySystem = new PIDFSystem(driveGain.getA(), driveGain.getB(), driveGain.getC(), driveGain.getD());
        this.rSystem = new PIDFSystem(rotGain.getA(), rotGain.getB(), rotGain.getC(), rotGain.getD());
        this.robotVelocity = robotVelocity;
        this.minimums = minimums;
    }

    @Override
    public Vector3 getVelocities() {
        return velocity;
    }

    @Override
    public void update(SensorData sensors, HardwareData hardware) {
        setTarget();
        double errY = localTarget.getB() - position.getB();
        double errX = localTarget.getA() - position.getA();
        double errR = MathUtils.getRadRotDist(position.getC(), Math.toRadians(localTarget.getC()));
        double r = Math.sqrt((errX * errX) + (errY * errY));
        double theta = Math.atan2(errY, errX) + position.getC();

        errR = errR / Math.toRadians(90);

        double x = (Math.cos(theta));
        double y = (Math.sin(theta));

        if(Math.abs(errX) < slowdownRadius){
            x *= (Math.abs(errX)/slowdownRadius);
        }
        if(Math.abs(errY) < slowdownRadius){
            y *= (Math.abs(errY)/slowdownRadius);
        }
        if(Math.abs(errR) < Math.toRadians(slowdownAngle)){
            errR *= (Math.abs(errR)/Math.toRadians(slowdownAngle));
        }

        double velR = Math.sqrt((robotVelocity.getA() * robotVelocity.getA()) + (robotVelocity.getB() * robotVelocity.getB()));
        double velTheta = Math.atan2(robotVelocity.getB(), robotVelocity.getA()) - position.getC();

        //double xCorr = xSystem.getCorrection((x * power) - ((velR * Math.cos(velTheta))/RobotConstants.UltimateGoal.MAX_X_SPEED), (y * power));
        //double yCorr = ySystem.getCorrection((-y * power) - ((-velR * Math.sin(velTheta))/RobotConstants.UltimateGoal.MAX_Y_SPEED), (x * power));
        //double rCorr = rSystem.getCorrection((errR * power) - (robotVelocity.getC()/RobotConstants.UltimateGoal.MAX_ROTATION_SPEED), 0);

        double xCorr = xSystem.getCorrection(r * Math.cos(theta), 0);
        double yCorr = ySystem.getCorrection(r * Math.sin(theta), 0);
        double rCorr = rSystem.getCorrection(errR, 0);

        //RobotLog.ii("Correction", (errR * power) + " | " + rCorr);

        velocity.set(xCorr * power, -yCorr * power, rCorr * power);
    }

    public abstract void setTarget();
}
