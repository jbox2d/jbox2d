package dynamics.contacts;

import common.Vec2;

public class ContactConstraintPoint {
	Vec2 localAnchor1;
	Vec2 localAnchor2;
	float normalImpulse;
	float tangentImpulse;
	float positionImpulse;
	float normalMass;
	float tangentMass;
	float separation;
	float velocityBias;
}
