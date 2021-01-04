package MathUtils;

public class Matrix33 {
    private Vector3 i, j, k;
    public Matrix33(Vector3 i, Vector3 j, Vector3 k){
        this.i = i;
        this.j = j;
        this.k = k;
    }

    public Matrix33(double... params){
        i = new Vector3(params[0], params[3], params[6]);
        j = new Vector3(params[1], params[4], params[7]);
        k = new Vector3(params[2], params[5], params[8]);
    }

    public Matrix33(Vector3 v){
        i = new Vector3(v.getA(), 0, 0);
        j = new Vector3(0, v.getB(), 0);
        k = new Vector3(0, 0, v.getC());
    }

    public Vector3 transform(Vector3 v){
        return i.scale(v.getA()).add(j.scale(v.getB()).add(k.scale(v.getC())));
    }
}
