package Motion.CrosstrackDrive;

import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;
import Motion.NormalDriveToPoint.NormalDriveToPoint;
import State.StateMachine;

public class CrosstrackDrive extends NormalDriveToPoint {
    private Vector2 startPos;
    private double kf, slowMod, mins;
    public CrosstrackDrive(StateMachine stateMachine, Vector3 position, Vector3 target, Vector2 startPos, double power, double kf, double slowMod, double mins) {
        super(stateMachine, position, target, power);
        this.startPos = startPos;
        this.kf = kf;
        this.slowMod = slowMod;
        this.mins = mins;
    }

    @Override
    public void setTarget() {
        Vector2 crosstrack = MathUtils.getClosestPoint(startPos, target.getVector2(), position.getVector2());
        double weight = position.getVector2().distanceTo(crosstrack) * kf;
        weight = MathUtils.clamp(weight, 0, 1);

        double weightedX = ((crosstrack.getA() - target.getA()) * weight) + target.getA();
        double weightedY = ((crosstrack.getB() - target.getB()) * weight) + target.getB();

        localTarget.set(weightedX, weightedY, target.getC());
        this.slowMod = MathUtils.clamp(slowMod/position.getVector2().distanceTo(target.getVector2()), mins, 1);
    }
}
