package dynamics.contacts;

import collision.Manifold;
import common.Vec2;
import common.Settings;
import dynamics.Body;

public class ContactConstraint {
    ContactConstraintPoint points[];

    Vec2 normal;

    Manifold manifold;

    Body body1;

    Body body2;

    float friction;

    float restitution;

    int pointCount;

    public ContactConstraint() {
        points = new ContactConstraintPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ContactConstraintPoint();
        }
        normal = new Vec2();
        manifold = new Manifold();
    }
}
