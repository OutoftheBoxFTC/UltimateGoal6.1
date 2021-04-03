package Hardware.HarwareUtils;

import MathSystems.MathUtils;

public class UGUtils {
    public static double getTurretValue(double angle){
        angle = MathUtils.clamp(angle, -20, 20);
        return (((angle + 20.0) / 40.0) * 0.7) + 0.15;
    }
}
