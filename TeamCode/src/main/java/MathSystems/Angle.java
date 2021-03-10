package MathSystems;


public class Angle {
    private double angle;
    private Angle(double angle){
        this.angle = angle;
    }

    public double degrees(){
        return Math.toDegrees(angle);
    }

    public double radians(){
        return angle;
    }

    public static Angle degrees(double angle){
        return new Angle(Math.toRadians(angle));
    }

    public static Angle radians(double angle){
        return new Angle(angle);
    }

    @Override
    public String toString() {
        return angle + " Randians | " + Math.toDegrees(angle) + " Degrees";
    }
}
