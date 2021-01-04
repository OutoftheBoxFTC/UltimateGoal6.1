package MathUtils;

public class Vector2 {
    private static final Vector2 ZERO = new Vector2(0, 0);

    private double a, b;
    public Vector2(double a, double b){
        this.a = a;
        this.b = b;
    }

    /**
     * takes the first two elements of a 3d vector and creates a 2d vector from them
     * @param v
     */
    public Vector2(Vector3 v) {
        a = v.getA();
        b = v.getB();
    }

    public void set(Vector2 v){
        this.a = v.getA();
        this.b = v.getB();
    }

    public Vector3 toVector3(double c){
        return new Vector3(a, b, c);
    }

    public void setA(double a) {
        this.a = a;
    }

    public void setB(double b) {
        this.b = b;
    }

    public double getA() {
        return a;
    }

    public double getB() {
        return b;
    }

    public Vector2 scale(double scalar){
        return new Vector2(a *scalar, b *scalar);
    }

    public Vector2 add(Vector2 v){
        return new Vector2(v.a + a, v.b + b);
    }

    public Vector2 clone(){
        return new Vector2(a, b);
    }

    public double length() {
        return Math.sqrt(a*a+b*b);
    }

    public double angleTo(Vector2 v){
        Vector2 d = add(v.scale(-1)).normalize();
        double angle = Math.acos(d.getA());
        if(d.getB()<0){
            return Math.PI*2-angle;
        }
        return angle;
    }

    public Vector2 normalize() {
        return scale(1/length());
    }

    public static Vector2 ZERO(){
        return ZERO.clone();
    }

    public double distanceTo(Vector2 vector){
        return Math.sqrt(Math.pow(vector.a - a, 2) + Math.pow(vector.b - b, 2));
    }

    public String toString(){
        return "X: " + a + " Y: " + b;
    }
}