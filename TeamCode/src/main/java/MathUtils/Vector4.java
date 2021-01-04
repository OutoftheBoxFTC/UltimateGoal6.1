package MathUtils;

public class Vector4 {
    private double a, b, c, d;
    public Vector4(double a, double b, double c, double d){
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public double getA() {
        return a;
    }

    public double getB() {
        return b;
    }

    public double getC() {
        return c;
    }

    public double getD() {
        return d;
    }

    public void setA(double a) {
        this.a = a;
    }

    public void setB(double b) {
        this.b = b;
    }

    public void setC(double c) {
        this.c = c;
    }

    public void setD(double d) {
        this.d = d;
    }

    public void set(Vector4 v){
        this.a = v.a;
        this.b = v.b;
        this.c = v.c;
        this.d = v.d;
    }

    public Vector4 add(Vector4 v){
        return new Vector4(v.a+a, v.b+b, v.c+c, v.d+d);
    }

    public Vector4 scale(double scalar){
        return new Vector4(a*scalar, b*scalar, c*scalar, d*scalar);
    }

    public Vector4 clone(){
        return scale(1);
    }

    @Override
    public String toString() {
        return a + ", " + b + ", " + c + ", " + d;
    }

    public static Vector4 ZERO(){
        return new Vector4(0, 0, 0, 0);
    }
}
