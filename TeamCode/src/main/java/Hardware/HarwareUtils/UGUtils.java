package Hardware.HarwareUtils;

import MathSystems.MathUtils;

public class UGUtils {
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, -24, 20);
        return (((angle + 24.0) / 48.0) * 0.95) + 0.05;
    }

    public static boolean inRange(double angle){
        return (angle < 20) && (angle > -24);
    }
}
