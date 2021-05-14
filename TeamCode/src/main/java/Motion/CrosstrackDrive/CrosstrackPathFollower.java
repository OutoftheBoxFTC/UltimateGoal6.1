package Motion.CrosstrackDrive;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;

import Hardware.Packets.HardwareData;
import Hardware.Packets.SensorData;
import Hardware.Robots.RobotConstants;
import MathSystems.Angle;
import MathSystems.MathUtils;
import MathSystems.Vector2;
import MathSystems.Vector3;
import Motion.Path.Path;
import State.StateMachine;
import State.VelocityDriveState;

public class CrosstrackPathFollower extends VelocityDriveState {
    private Path path;
    private Vector3 position, velocity;
    private ArrayList<Vector2> pathLines = new ArrayList<>();
    private double rotTol, speed, kf, pathLength;

    public CrosstrackPathFollower(StateMachine stateMachine, Vector3 position, Path path, double rotTol, double speed, double kf) {
        super(stateMachine);
        this.position = position;
        this.path = path;
        this.rotTol = rotTol;
        this.speed = speed;
        this.kf = kf;
        this.pathLines = path.getPathLines();
        this.velocity = Vector3.ZERO();
        this.pathLength = MathUtils.arcLength(path.getPathLines());
    }

    public CrosstrackPathFollower(StateMachine stateMachine, Vector3 position, Path path, double rotTarget){
        this(stateMachine, position, path, rotTarget, 1, 0.5);
    }

    public CrosstrackPathFollower(StateMachine stateMachine, Vector3 position, Path path){
        this(stateMachine, position, path, 0, 1, 0.15);
    }

    @Override
    public Vector3 getVelocities() {
        return velocity;
    }

    @Override
    public void update(SensorData sensorData, HardwareData hardwareData) {
        double minDist = Double.MAX_VALUE;
        Vector2 closestPoint = Vector2.ZERO();
        Vector3 target = Vector3.ZERO();
        Vector3 locstart = Vector3.ZERO();
        int idx = 0;
        ArrayList<Double> dists = new ArrayList<>();
        for(int i = 1; i < path.getPathLines().size(); i ++) {
            Vector2 tmp = MathUtils.getClosestPoint(path.getPathLines().get(i-1), path.getPathLines().get(i), position.getVector2());
            dists.add(tmp.distanceTo(position.getVector2()));
            if(tmp.distanceTo(position.getVector2()) - minDist <= 1E-3) {
                minDist = tmp.distanceTo(position.getVector2());
                closestPoint.set(tmp);
                target.set(path.getPathLines().get(i), 0);
                locstart.set(path.getPathLines().get(i-1), 0);
                idx = i;
            }
        }

        Vector2 tang = target.subtract(locstart).getVector2().normalize();

        Vector2 mainVel = getLinVel(locstart, target, closestPoint);

        ArrayList<Vector2> pathLntArr = new ArrayList<>();
        pathLntArr.add(position.getVector2());
        pathLntArr.addAll(path.getPathLines().subList(idx, path.getPathLines().size()));

        double maxVel = Math.sqrt(2 * RobotConstants.UltimateGoal.MAX_LIN_ACCEL * MathUtils.arcLength(pathLntArr));


        maxVel = MathUtils.clamp(maxVel, 0, RobotConstants.UltimateGoal.MAX_SPEED * speed);

        mainVel = mainVel.scale(maxVel);


        ArrayList<Vector2> pathList = new ArrayList<>();
        pathList.addAll(path.getPathLines().subList(idx, path.getPathLines().size()));
        pathList.add(0, position.getVector2());

        double rotWeight = MathUtils.arcLength(pathList)/pathLength;
        double rotTarget = path.getLines().get(idx).getC();

        double maxRotSpeed = Math.sqrt(2 * RobotConstants.UltimateGoal.MAX_R_ACCEL * Math.abs(MathUtils.getRadRotDist(position.getC(), rotTarget)));
        maxRotSpeed = MathUtils.clamp(maxRotSpeed, 0, RobotConstants.UltimateGoal.MAX_ROTATION_SPEED * speed);

        double rSign = MathUtils.sign(MathUtils.getRadRotDist(position.getC(), rotTarget));

        Vector2 rotatedVels = mainVel.rotate(Angle.radians(-(position.getC() + (maxRotSpeed * rSign * RobotConstants.UltimateGoal.rotFF))));

        if(mainVel.length() == 0) {
            rSign = 0;
        }
        rotatedVels.setB(rotatedVels.getB() * -1);
        rotatedVels.set(rotatedVels.scale(1/RobotConstants.UltimateGoal.MAX_SPEED));
        maxRotSpeed *= 1/RobotConstants.UltimateGoal.MAX_ROTATION_SPEED;
        RobotLog.ii("Vel", rotatedVels.toString() + " | " + mainVel + " | " + tang + " | " + closestPoint + " | " + target);

        if(Math.abs(rotatedVels.getA()) < RobotConstants.UltimateGoal.KF){
            rotatedVels.setA(RobotConstants.UltimateGoal.KF * MathUtils.sign(rotatedVels.getA()));
        }

        if(Math.abs(rotatedVels.getB()) < RobotConstants.UltimateGoal.KF){
            rotatedVels.setB(RobotConstants.UltimateGoal.KF * MathUtils.sign(rotatedVels.getB()));
        }

        double rotSpeed = Math.max(maxRotSpeed, RobotConstants.UltimateGoal.KF) * MathUtils.sign(MathUtils.getRadRotDist(position.getC(), rotTarget));
        if(Math.abs(MathUtils.getRadRotDist(position.getC(), rotTarget)) < Math.toRadians(rotTol)){
            rotSpeed = 0;
        }
        velocity.set(rotatedVels, rotSpeed);
    }

    public Vector2 getLinVel(Vector3 start, Vector3 target, Vector2 closestPoint) {
        Vector2 tang = target.subtract(start).getVector2().normalize();
        if(closestPoint.distanceTo(position.getVector2()) < 0.1) {
            tang = target.subtract(position).getVector2().normalize();
        }
        tang = target.subtract(position).getVector2().normalize();

        Vector2 norm = closestPoint.subtract(position.getVector2()).normalize();

        double weight = closestPoint.distanceTo(position.getVector2()) * kf;

        weight = MathUtils.clamp(weight, 0, 1);

        Vector2 localTarget = norm.subtract(tang).scale(weight).add(tang);
        //System.out.println(weight + " | " + localTarget + " | " + norm + " | " + tang);
        return localTarget.normalize();
    }
}
