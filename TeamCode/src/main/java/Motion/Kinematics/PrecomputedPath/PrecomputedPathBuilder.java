package Motion.Kinematics.PrecomputedPath;

import Motion.Kinematics.PurePursuit.*;
import State.*;
import MathSystems.*;

/**
 * Precomputed Path Builder
 * This builder creates a PurePursuit class, adding all the precomputed path vectors
 */

public class PrecomputedPathBuilder extends PurePursuitBuilder {
    public PrecomputedPathBuilder(StateMachine stateMachine, Vector3 position) {
        super(stateMachine, position);
    }
    public PrecomputedPathBuilder setPathString(String s){
        String[] arr = s.split("/");
        for(String str : arr){
            this.addTarget(new Vector2(Double.parseDouble(str.split(",")[0]), Double.parseDouble(str.split(",")[1])));
        }
        return this;
    }

    public Vector2 getFinalTarget(){
        if(this.targets.size() > 0){
            return targets.get(targets.size()-1);
        }
        return Vector2.ZERO();
    }
}
