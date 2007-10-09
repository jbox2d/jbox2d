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
        col1 = c1;
        col2 = c2;
    }

    public void set(float angle) {
        float c = (float) Math.cos(angle);
        float s = (float) Math.sin(angle);
        col1.x = c;
        col2.x = -s;
        col1.y = s;
        col2.y = c;
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

    public Vec2 mulT(Vec2 v) {
        return new Vec2(Vec2.dot(v, col1), Vec2.dot(v, col2));
    }

    public Mat22 add(Mat22 B) {
        return new Mat22(col1.add(B.col1), col2.add(B.col2));
    }
}
