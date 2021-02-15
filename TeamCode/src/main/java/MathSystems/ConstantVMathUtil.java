package MathSystems;

/**
 * For a breakdown of how this works
 *
 * https://www.desmos.com/calculator/lzxnrmvsnd
 *
 * For the derivation of these equations (Requires a bit of calculus knowledge)
 *
 * https://www.desmos.com/calculator/ezczsdozdt
 *
 * For a video explanation of this algorithm:
 * https://www.youtube.com/watch?v=YcZqutpT1r8
 */
public class ConstantVMathUtil {

    private static double displacementROC(double x, double y){
        double num = (Math.pow(x, 2) + Math.pow(y, 2));
        double den = Math.sqrt((Math.pow(x, 2) * 1) + (Math.pow(y, 2) * 1));
        return num/den;
    }

    private static double xIntegral(double x, double y, double rot){
        double out = ((displacementROC(x, y) * Math.cos(rot - Math.atan2(y, x))) - x)/rot;
        return out;
    }

    private static double yIntegral(double x, double y, double rot){
        double out = ((displacementROC(x, y) * Math.sin(rot - Math.atan2(y, x))) + y)/rot;
        return out;
    }

    private static Vector2 toPolar(double x, double y){
        double r = Math.sqrt((x * x) + (y * y));
        double theta = Math.atan2(y, x);
        return new Vector2(r, theta);
    }

    private static Vector2 toCartesian(double r, double theta){
        return new Vector2(r * Math.cos(theta), r * Math.sin(theta));
    }

    private static double getQuadrent(double x, double y){
        if(x < 0 && y > 0){
            return Math.PI/2;
        }
        if(x < 0 && y < 0){
            return Math.PI;
        }
        if(x > 0 && y < 0){
            return (3 * Math.PI)/2;
        }
        return 0;
    }

    public static Vector2 toRobotCentric(double forward, double strafe, double rot){
        if(Math.abs(rot) <= 1.0E-4){
            return new Vector2(strafe, forward);
        }
        Vector2 vector = new Vector2(xIntegral(forward, strafe, rot), yIntegral(forward, strafe, rot));
        Vector2 polar = toPolar(vector.getA(), vector.getB());
        if(Double.isNaN(vector.getA()) || Double.isNaN(vector.getB())){
            return new Vector2(strafe, forward);
        }
        return vector;
    }
}