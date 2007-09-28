package dynamics.contacts;

import collision.Manifold;
import common.Vec2;
import dynamics.Body;

public class ContactConstraint {
	ContactConstraintPoint points[]; //FIXME ContactConstraintPoint points[Settings.maxManifoldPoints]
	Vec2 normal;
	Manifold manifold;
	Body body1;
	Body body2;
	float friction;
	float restitution;
	int pointCount;
}
