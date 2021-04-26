package Motion.CrosstrackDrive;

import MathSystems.Vector3;
import Motion.Path.Path;
import State.StateMachine;

public class CrosstrackBuilder {
    private StateMachine stateMachine;
    private Vector3 position;
    public CrosstrackBuilder(StateMachine stateMachine, Vector3 position){
        this.stateMachine = stateMachine;
        this.position = position;
    }

    public CrosstrackPathFollower follow(Path path){
        return new CrosstrackPathFollower(stateMachine, position, path);
    }

    public CrosstrackPathFollower follow(Path path, double rotTarget){
        return new CrosstrackPathFollower(stateMachine, position, path, rotTarget);
    }

    public CrosstrackPathFollower follow(Path path, double rotTol, double speed, double kf){
        return new CrosstrackPathFollower(stateMachine, position, path, rotTol, speed, kf);
    }
}
