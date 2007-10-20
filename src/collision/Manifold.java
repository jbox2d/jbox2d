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
        // FIXME? Need to check how C++ handles an implicit
        // copy of a Manifold, by copying the points array
        // or merely passing a pointer to it.
        System.arraycopy(other.points, 0, points, 0, other.points.length);
        // points = new ContactPoint[other.points.length];
        // for (int i=0; i<other.points.length; i++){
        // points[i] = new ContactPoint(other.points[i]);
        // }
        normal = other.normal.clone();
        pointCount = other.pointCount;// points.length;
    }
}
