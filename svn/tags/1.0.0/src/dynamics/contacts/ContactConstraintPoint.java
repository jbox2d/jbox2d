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

    public ContactConstraintPoint() {
        // Probably unnecessary to init
        localAnchor1 = new Vec2();
        localAnchor2 = new Vec2();
    }
}
