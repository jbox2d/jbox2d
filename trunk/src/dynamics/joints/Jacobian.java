package dynamics.joints;

import common.Vec2;

public class Jacobian {
    Vec2 linear1;

    float angular1;

    Vec2 linear2;

    float angular2;

    public Jacobian() {
        linear1 = new Vec2();
        linear2 = new Vec2();
        setZero();
    }

    void setZero() {
        linear1.setZero();
        angular1 = 0.0f;
        linear2.setZero();
        angular2 = 0.0f;
    }

    void set(Vec2 x1, float a1, Vec2 x2, float a2) {
        linear1 = x1;
        angular1 = a1;
        linear2 = x2;
        angular2 = a2;
    }

    float Compute(Vec2 x1, float a1, Vec2 x2, float a2) {
        return Vec2.dot(linear1, x1) + angular1 * a1 + Vec2.dot(linear2, x2)
                + angular2 * a2;
    }
}
