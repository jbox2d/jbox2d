package collision;

import common.Settings;
import common.Vec2;

public class Manifold {
    public ContactPoint[] points;

    public Vec2 normal;

    public int pointCount;

    public Manifold() {
        points = new ContactPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ContactPoint();
        }
        normal = new Vec2();
        pointCount = 0;
    }

    public Manifold(Manifold other) {
        points = new ContactPoint[Settings.maxManifoldPoints];
        System.arraycopy(other.points, 0, points, 0, other.points.length);
        normal = other.normal.clone();
        pointCount = other.pointCount;// points.length;
    }

}
