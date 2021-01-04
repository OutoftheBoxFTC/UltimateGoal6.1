package MathUtils;

public class Matrix43 {
    private Vector4 i, j, k;
    public Matrix43(Vector4 i, Vector4 j, Vector4 k){
        this.i = i;
        this.j = j;
        this.k = k;
    }

    public Matrix43(double... params){
        i = new Vector4(params[0], params[3], params[6], params[9]);
        j = new Vector4(params[1], params[4], params[7], params[10]);
        k = new Vector4(params[2], params[5], params[8], params[11]);
    }

    public Vector4 transform(Vector3 v){
        return i.scale(v.getA()).add(j.scale(v.getB())).add(k.scale(v.getC()));
    }

    public Matrix43 scale(double scalar){
        return new Matrix43(i.scale(scalar), j.scale(scalar), k.scale(scalar));
    }
}
