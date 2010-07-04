package collision;

import common.Vec2;

public class ContactPoint {
    public Vec2 position;

    public float separation;

    public float normalImpulse;

    public float tangentImpulse;

    public ContactID id;

    public ContactPoint() {
        position = new Vec2();
        separation = normalImpulse = tangentImpulse = 0f;
        id = new ContactID();
    }

    public ContactPoint(ContactPoint cp) {
        position = cp.position.clone();
        separation = cp.separation;
        normalImpulse = cp.normalImpulse;
        tangentImpulse = cp.tangentImpulse;
        id = new ContactID(cp.id);
    }

}
