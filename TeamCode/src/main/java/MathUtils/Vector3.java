package MathUtils;

public class Vector3 {
    private static final Vector3 ZERO = new Vector3(0, 0, 0);

    private double a, b, c;
    public Vector3(double a, double b, double c){
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public Vector3(Vector2 v, double c){
        a = v.getA();
        b = v.getB();
        this.c = c;
    }

    public double getC() {
        return c;
    }

    public void setC(double c) {
        this.c = c;
    }

    public double getA() {
        return a;
    }

    public void setA(double a) {
        this.a = a;
    }

    public double getB() {
        return b;
    }

    public void setB(double b) {
        this.b = b;
    }

    public void set(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public Vector3 clone(){
        return new Vector3(a, b, c);
    }

    public Vector3 subtract(Vector3 v) {
        return new Vector3(a-v.a, b-v.b, c-v.c);
    }

    public Vector3 add(Vector3 v){
        return new Vector3(v.a+a, v.b+b, v.c+c);
    }

    public Vector3 add(double a, double b, double c){
        return new Vector3(a+a, b+b, c+c);
    }

    public Vector3 scale(double scalar){
        return new Vector3(a*scalar, b*scalar, c*scalar);
    }

    public void set(Vector3 v) {
        a = v.a;
        b = v.b;
        c = v.c;
    }

    public void set(Vector2 v, double c) {
        this.a = v.getA();
        this.b = v.getB();
        this.c = c;
    }

    public Vector2 getVector2(){
        return new Vector2(a, b);
    }

    public static Vector3 ZERO(){
        return ZERO.clone();
    }

    public boolean equals(Vector3 vector3){
        return (vector3.a == a) && (vector3.b == b) && (vector3.c == c);
    }

    public Vector4 getVector4(double d){
        return new Vector4(a, b, c, d);
    }

    @Override
    public String toString() {
        return a + ", " + b + ", " + c;
    }
}