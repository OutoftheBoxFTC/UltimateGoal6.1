package MathSystems;

import java.util.ArrayList;
import java.util.List;

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

    public static double sign(double in){
        if(in == 0) {
            return 1;
        }
        return in/Math.abs(in);
    }

    public static ArrayList<Vector2> approxCurve(Vector2 start, Vector2 end, Vector2 control, double numSegments){
        ArrayList<Vector2> coordinates = new ArrayList<Vector2>();

        coordinates.add(start);

        double s  = 0;
        double t = 1;

        while (s < t) {
            s += 1/numSegments;
            double controlParameter = (1 - s);
            Vector2 Q_0 = new Vector2(controlParameter * start.getA(), controlParameter * start.getB()).add(new Vector2(s * control.getA(), s * control.getB()));
            Vector2 Q_1 = new Vector2(controlParameter * control.getA(), controlParameter * control.getB()).add(new Vector2(s * end.getA(), s * end.getB()));
            Vector2 R_0 = new Vector2(controlParameter * Q_0.getA(), controlParameter * Q_0.getB()).add(new Vector2(s * Q_1.getA(), s * Q_1.getB()));
            coordinates.add(R_0);
        }
        coordinates.remove(coordinates.size()-1);
        coordinates.add(end);
        return coordinates;
    }

    public static Vector2 getClosestPoint(Vector2 line1point1, Vector2 line1point2, Vector2 line2point){
        double dx = line1point1.getA() - line1point2.getA();
        double dy = line1point1.getB() - line1point2.getB();

        double line1slope = 0;

        double x = 0;
        double y = 0;

        if(dx == 0 && dy == 0){
            return line1point1;
        }else if(dx == 0){
            x = line1point1.getA();
            y = line2point.getB();
            //return new Vector2(line1point1.getA(), line2point.getB());
        }else if(dy == 0){
            x = line2point.getA();
            y = line1point1.getB();
            //return new Vector2(line2point.getA(), line1point1.getB());
        }else{
            line1slope = dy/dx;

            double line2slope = -1/line1slope;

            x = ((-line2slope * line2point.getA()) + line2point.getB() + (line1slope * line1point1.getA()) - line1point1.getB())/(line1slope - line2slope);

            y = line1slope * (x - line1point1.getA()) + line1point1.getB();
        }

        double minX = Math.min(line1point1.getA(), line1point2.getA());
        double maxX = Math.max(line1point1.getA(), line1point2.getA());

        double minY = Math.min(line1point1.getB(), line1point2.getB());
        double maxY = Math.max(line1point1.getB(), line1point2.getB());

        if(x < minX || x > maxX || y < minY || y > maxY) {
            if(line1point1.distanceTo(line2point) < line1point2.distanceTo(line2point)) {
                return line1point1;
            }else {
                return line1point2;
            }
        }

        return new Vector2(x, y);
    }

    public static double arcLength(ArrayList<Vector2> lines) {
        double dist = 0;
        for(int i = 1; i < lines.size(); i ++) {
            dist += lines.get(i-1).distanceTo(lines.get(i));
        }
        return dist;
    }

    public static double arcLength(List<Vector2> lines) {
        double dist = 0;
        for(int i = 1; i < lines.size(); i ++) {
            dist += lines.get(i-1).distanceTo(lines.get(i));
        }
        return dist;
    }

    public static Vector2 toPolar(Vector2 pos){
        return toPolar(pos.getA(), pos.getB());
    }

    public static double clamp(double val, double min, double max){
        return Math.max(min, Math.min(max, val));
    }

    public static Vector2 toCartesian(double r, double theta){
        return new Vector2(r * Math.cos(theta), r * Math.sin(theta));
    }

    public static Vector2 toCartesian(Vector2 pos){
        return toCartesian(pos.getA(), pos.getB());
    }

    public static double millisToSec(long time){
        return (time/1000.0);
    }

    public static long secToMillis(long time){
        return time * 1000;
    }

    public static long secToNano(long time){
        return time * ((long)1_000_000_000);
    }

    public static long millisToNano(long time){
        return secToNano((long)millisToSec(time));
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
