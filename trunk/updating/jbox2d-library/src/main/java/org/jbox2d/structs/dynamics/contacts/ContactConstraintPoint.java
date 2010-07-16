package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.common.Vec2;

// updated to rev 100

public class ContactConstraintPoint {
	
    public final Vec2 localPoint;
    public final Vec2 rA;
    public final Vec2 rB;

    public float normalImpulse;
    public float tangentImpulse;
    public float normalMass;
    public float tangentMass;
    public float velocityBias;

    public ContactConstraintPoint() {
        localPoint = new Vec2();
        rA = new Vec2();
        rB = new Vec2();
    }
    
    public void set(final ContactConstraintPoint cp){
    	localPoint.set(cp.localPoint);
    	rA.set(cp.rA);
    	rB.set(cp.rB);
    	normalImpulse = cp.normalImpulse;
    	tangentImpulse = cp.tangentImpulse;
    	normalMass = cp.normalMass;
    	tangentMass = cp.tangentMass;
        velocityBias = cp.velocityBias;
    }
}