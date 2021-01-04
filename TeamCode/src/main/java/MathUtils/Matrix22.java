package MathUtils;

public class Matrix22 {
    private Vector2 i, j;
    public Matrix22(double a, double b, double c, double d){
        i = new Vector2(a, c);
        j = new Vector2(b, d);
    }

    public Matrix22(Vector2 i, Vector2 j){
        this.i = i;
        this.j = j;
    }

    public Matrix22(Vector2 v){
        i = new Vector2(v.getA(), 0);
        j = new Vector2(0, v.getB());
    }

    public Vector2 transform(Vector2 v){
        return i.scale(v.getA()).add(j.scale(v.getB()));
    }

    public Matrix22 compose(Matrix22 m){
        return new Matrix22(transform(m.i), transform(m.j));
    }

    public Matrix22 scale(double scalar) {
        return new Matrix22(i.scale(scalar), j.scale(scalar));
    }

    public double determinate(){
        return (i.getA()*j.getB())-(i.getB()*j.getA());
    }

    public Matrix22 inverse(){
        return new Matrix22(j.getB(), i.getB(), j.getA(), i.getA()).scale(1/determinate());
    }
}
