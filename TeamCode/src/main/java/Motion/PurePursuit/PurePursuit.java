package Motion.PurePursuit;

import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;

import Motion.DriveToPoint.DriveToPoint;
import State.StateMachine;
import MathUtils.*;

/**
 * Drives in a curve using the Pure Pursuit driving system
 * Essentially Pure Pursuit uses a circle around the robot intersecting with the path to compute motion
 */

public class PurePursuit extends DriveToPoint {
    private double radius, rotTarget;
    private ArrayList<Vector2> targets;
    private int index = 0;
    public PurePursuit(StateMachine stateMachine, Vector3 position, double power, double radius, ArrayList<Vector2> targets, double rotTarget) {
        super(stateMachine, position, Vector3.ZERO(), power);
        this.radius = radius;
        this.targets = new ArrayList<>();
        this.targets.addAll(targets);
        this.rotTarget = rotTarget;
    }

    @Override
    public void setTarget() {
        double locRadius = radius;
        Vector2 intersect1 = Vector2.ZERO(), intersect2 = Vector2.ZERO();
        Vector2 pos = position.getVector2();
        Vector2 targetPos = Vector2.ZERO();
        //RobotLog.ii("Position", position.toString());
        Vector2[] locTargets = getBestLine();
        double intersections = findLineCircleIntersections(position.getA(), position.getB(), locRadius, locTargets[0], locTargets[1], intersect1, intersect2);
        while(intersections < 2){
            radius += 0.1;
            intersections = findLineCircleIntersections(position.getA(), position.getB(), locRadius, locTargets[0], locTargets[1], intersect1, intersect2);
        }
        if(intersect1.distanceTo(pos) < intersect2.distanceTo(pos)){
            targetPos.set(intersect1);
        }else{
            targetPos.set(intersect2);
        }
        //RobotLog.i(targetPos.toString());
        localTarget.set(targetPos, rotTarget);
    }

    private Vector2[] getBestLine(){
        HashMap<Double, Integer> distMap = new HashMap<>();
        Vector2 intersect1 = Vector2.ZERO(), intersect2 = Vector2.ZERO();
        double locRadius = radius;
        for(int i = 0; i < (targets.size()-1); i ++){
            if(targets.get(i).distanceTo(position.getVector2()) <= radius) {
                double intersections = findLineCircleIntersections(position.getA(), position.getB(), locRadius, targets.get(index), targets.get(index + 1), intersect1, intersect2);
                if (intersections == 1) {
                    distMap.put(intersect1.distanceTo(targets.get(targets.size() - 1)), i);
                } else {
                    distMap.put(Math.min(intersect1.distanceTo(targets.get(targets.size() - 1)), intersect2.distanceTo(targets.get(targets.size() - 1))), i);
                }
            }
        }
        double min = Double.POSITIVE_INFINITY;
        for(double d : distMap.keySet()){
            min = Math.min(d, min);
        }
        return new Vector2[]{targets.get(distMap.get(min)), targets.get(distMap.get(min)+1)};
    }

    private double findLineCircleIntersections(
            double cx, double cy, double radius,
            Vector2 point1, Vector2 point2, Vector2 intersect1, Vector2 intersect2) {
        double dx, dy, A, B, C, det, t;

        dx = point2.getA() - point1.getA();
        dy = point2.getB() - point1.getB();

        A = dx * dx + dy * dy;
        B = 2 * (dx * (point1.getA() - cx) + dy * (point1.getB() - cy));
        C = (point1.getA() - cx) * (point1.getA() - cx) +
                (point1.getB() - cy) * (point1.getB() - cy) -
                radius * radius;

        det = B * B - 4 * A * C;
        if ((A <= 0.0000001) || (det < 0)) {
            // No real solutions.
            return 0;
        } else if (det == 0) {
            // One solution.
            t = -B / (2 * A);
            Vector2 intersection1 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            Vector2 intersection2 = new Vector2(Double.NaN, Double.NaN);
            intersect1.set(intersection1);
            intersect2.set(intersection2);
            return 1;
        } else {
            // Two solutions.
            t = (float) ((-B + Math.sqrt(det)) / (2 * A));
            Vector2 intersection1 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            t = (float) ((-B - Math.sqrt(det)) / (2 * A));
            Vector2 intersection2 =
                    new Vector2(point1.getA() + t * dx, point1.getB() + t * dy);
            intersect1.set(intersection1);
            intersect2.set(intersection2);
            return 2;
        }
    }
}
