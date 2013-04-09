package collision;

import common.Vec2;

public class MassData {
    public float mass;

    public Vec2 center;

    public float I;

    public MassData() {
        mass = I = 0f;
        center = new Vec2();
    }

}
