package collision;

import common.Vec2;

public class AABB {
    public Vec2 minVertex, maxVertex;

    public AABB(Vec2 minVertex, Vec2 maxVertex) {
        // super(); //ewjordan: no superclass
        this.minVertex = minVertex.clone(); // clone to be safe
        this.maxVertex = maxVertex.clone();
    }

    public AABB(AABB copy) {
        this(new Vec2(copy.minVertex.x, copy.minVertex.y), new Vec2(
                copy.maxVertex.x, copy.maxVertex.y));
    }

    public AABB() {
        minVertex = new Vec2();
        maxVertex = new Vec2();
    }

    boolean isValid() {
        Vec2 d = maxVertex.sub(minVertex);
        return d.x >= 0.0f && d.y >= 0 && minVertex.isValid()
                && maxVertex.isValid();
    }

    boolean testOverlap(AABB box) {
        Vec2 d1, d2;
        d1 = box.minVertex.sub(maxVertex);
        d2 = minVertex.sub(box.maxVertex);

        if (d1.x > 0.0f || d1.y > 0.0f || d2.x > 0.0f || d2.y > 0.0f) {
            return false;
        }

        return true;
    }
}
