package Motion.CrosstrackDrive;

import MathSystems.Vector2;
import MathSystems.Vector3;
import Motion.DriveToPoint.DriveToPointBuilder;
import State.StateMachine;

public class CrosstrackDriveBuilder {
    private StateMachine stateMachine;
    private double speed, rot, slowMod, minimums, kf;
    private Vector3 position;
    private Vector2 target, start;
    public CrosstrackDriveBuilder(StateMachine stateMachine, Vector3 position){
        this.position = position;
        this.stateMachine = stateMachine;
        target = Vector2.ZERO();
        start = Vector2.ZERO();
        speed = 1;
        rot = 0;
        slowMod = 25;
        minimums = 0.125;
        kf = 0.5;
    }

    public CrosstrackDriveBuilder setSpeed(double speed){
        this.speed = speed;
        return this;
    }

    public CrosstrackDriveBuilder setRot(double rot){
        this.rot = rot;
        return this;
    }

    public CrosstrackDriveBuilder setTarget(Vector2 target){
        this.target.set(target);
        return this;
    }

    public CrosstrackDriveBuilder setStart(Vector2 start){
        this.start = start;
        return this;
    }

    public CrosstrackDriveBuilder setSlowMod(double slowMod) {
        this.slowMod = slowMod;
        return this;
    }

    public CrosstrackDriveBuilder setMinimums(double minimums){
        this.minimums = minimums;
        return this;
    }

    public CrosstrackDriveBuilder setKf(double kf){
        this.kf = kf;
        return this;
    }

    public CrosstrackDrive complete(){
        return new CrosstrackDrive(stateMachine, position, target.toVector3(rot), start, speed, kf, slowMod, minimums);
    }
}
