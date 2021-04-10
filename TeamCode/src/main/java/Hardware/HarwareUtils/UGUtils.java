package Hardware.HarwareUtils;

import MathSystems.MathUtils;

public class UGUtils {
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, -21, 24);
        return (((angle + 21.0) / 45.0) * 0.95) + 0.05;
    }

    public static boolean inRange(double angle){
        return (angle < 24) && (angle > -21);
    }
}
