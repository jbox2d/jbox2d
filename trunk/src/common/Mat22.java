package common;

public class Mat22 {
    public Vec2 col1, col2;

    public Mat22() {
        this(new Vec2(), new Vec2());
    }

    public Mat22(float angle) {
        this();
        set(angle);
    }

    public Mat22(Vec2 c1, Vec2 c2) {
        //ewj: changed this to clone Vec2s for safety
        col1 = c1.clone();
        col2 = c2.clone();
    }
    
    public void set(Mat22 m) {
        col1.x = m.col1.x;
        col1.y = m.col1.y;
        col2.x = m.col2.x;
        col2.y = m.col2.y;
    }
    
    public Mat22 clone(){
        return new Mat22(this.col1.clone(),this.col2.clone());
    }
    
    public void SetIdentity() {
        col1.x = 1.0f; col2.x = 0.0f;
        col1.y = 0.0f; col2.y = 1.0f;
    }
    
    public void SetZero() {
        col1.x = 0.0f; col2.x = 0.0f;
        col1.y = 0.0f; col2.y = 0.0f;
    }

    public void set(float angle) {
        float c = (float) Math.cos(angle);
        float s = (float) Math.sin(angle);
        col1.x = c;
        col2.x = -s;
        col1.y = s;
        col2.y = c;
    }
    
    public void set(Vec2 c1, Vec2 c2) {
        col1.x = c1.x; col2.x = c2.x;
        col1.y = c1.y; col2.y = c2.y;
    }

    public Mat22 invert() {
        float a = col1.x, b = col2.x, c = col1.y, d = col2.y;
        Mat22 B = new Mat22();
        float det = a * d - b * c;
        // b2Assert(det != 0.0f);
        det = 1.0f / det;
        B.col1.x = det * d;
        B.col2.x = -det * b;
        B.col1.y = -det * c;
        B.col2.y = det * a;
        return B;
    }

    public Mat22 abs() {
        return new Mat22(col1.abs(), col2.abs());
    }

    public Vec2 mul(Vec2 v) {
        return new Vec2(col1.x * v.x + col2.x * v.y, col1.y * v.x + col2.y
                * v.y);
    }
    
    public Mat22 mul(Mat22 R) {
        Mat22 C = new Mat22();
        C.set(this.mul(R.col1),this.mul(R.col2));
        return C;
    }
    
    public Mat22 mulT(Mat22 B) {
        Vec2 c1 = new Vec2(Vec2.dot(this.col1,B.col1),Vec2.dot(this.col2,B.col1));
        Vec2 c2 = new Vec2(Vec2.dot(this.col1,B.col2),Vec2.dot(this.col2,B.col2));
        Mat22 C = new Mat22();
        C.set(c1,c2);
        return C;
    }

    public Vec2 mulT(Vec2 v) {
        return new Vec2(Vec2.dot(v, col1), Vec2.dot(v, col2));
    }

    public Mat22 add(Mat22 B) {
        return new Mat22(col1.add(B.col1), col2.add(B.col2));
    }
    
}
