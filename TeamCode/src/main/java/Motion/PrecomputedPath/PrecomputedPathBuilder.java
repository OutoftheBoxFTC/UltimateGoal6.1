package Motion.PrecomputedPath;

import android.util.Base64;

import Motion.PurePursuit.*;
import State.*;
import MathUtils.*;

/**
 * Precomputed Path Builder
 * This builder creates a PurePursuit class, adding all the precomputed path vectors
 */

public class PrecomputedPathBuilder extends PurePursuitBuilder {
    public PrecomputedPathBuilder(StateMachine stateMachine, Vector3 position) {
        super(stateMachine, position);
    }
    public PrecomputedPathBuilder setPathString(String s){
        byte[] decoded = Base64.decode(s, Base64.DEFAULT);
        String[] arr = new String(decoded).split("/");
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
