package MathUtils;

public class Matrix34 {
    private Vector3 i, j, k, l;
    public Matrix34(Vector3 i, Vector3 j, Vector3 k, Vector3 l){
        this.i = i;
        this.j = j;
        this.k = k;
        this.l = l;
    }

    public Matrix34(double... params){
        if(params.length==12) {
            i = new Vector3(params[0], params[4], params[8]);
            j = new Vector3(params[1], params[5], params[9]);
            k = new Vector3(params[2], params[6], params[10]);
            l = new Vector3(params[3], params[7], params[11]);
        }
    }

    public Vector3 transform(Vector4 v){
        return i.scale(v.getA()).add(j.scale(v.getB())).add(k.scale(v.getC())).add(l.scale(v.getD()));
    }
}
