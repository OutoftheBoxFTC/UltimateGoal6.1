package MathUtils;

/**
 * Contains some commonly used math functions, so they don't need to be repeatedly typed out
 */

public class MathUtils {
    public static Vector2 toPolar(double x, double y){
        double r = Math.sqrt((x * x) + (y * y));
        double theta = Math.atan2(y, x);
        return new Vector2(r, theta);
    }

    public static double getRadRotDist(double start, double end){
        double dist = end - start;
        if(Math.abs(dist) > Math.PI){
            if(start > Math.PI){
                return ((end + (2 * Math.PI)) - start);
            }else {
                return ((end - (2 * Math.PI)) - start);
            }
        }
        return dist;
    }

    public static Vector2 toPolar(Vector2 pos){
        return toPolar(pos.getA(), pos.getB());
    }

    public static Vector2 toCartesian(double r, double theta){
        return new Vector2(r * Math.cos(theta), r * Math.sin(theta));
    }

    public static Vector2 toCartesian(Vector2 pos){
        return toCartesian(pos.getA(), pos.getB());
    }

    public static long millisToSec(long time){
        return time/1000;
    }

    public static long secToMillis(long time){
        return time * 1000;
    }

    public static long secToNano(long time){
        return time * ((long)1_000_000_000);
    }

    public static long millisToNano(long time){
        return secToNano(millisToSec(time));
    }

    public static long nanoToMillis(long time){
        return secToMillis(nanoToSec(time));
    }

    public static long nanoToSec(long time){
        return time/((long)1_000_000_000);
    }

    public static double nanoToDSec(long time){
        return time/1_000_000_000.0;
    }
}
