package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.Manifold.ManifoldType;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

// updated to rev 100

public class ContactConstraint {
	public final ContactConstraintPoint points[];

 	public final Vec2 localPlaneNormal;
    public final Vec2 localPoint;
    public final Vec2 normal;
    
    public final Mat22 normalMass;
    public final Mat22 K;

    public Body bodyA;
    public Body bodyB;

    public ManifoldType type;
    
    public float radius;
    public float friction;
    public float restitution;
    public int pointCount;

    public Manifold manifold = null;

    public ContactConstraint() {
        points = new ContactConstraintPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ContactConstraintPoint();
        }
        pointCount = 0;
        localPlaneNormal = new Vec2();
        localPoint = new Vec2();
        normal = new Vec2();
        normalMass = new Mat22();
        K = new Mat22();

    }
    
    public void set(final ContactConstraint cp){
    	pointCount = cp.pointCount;
    	localPlaneNormal.set(cp.localPlaneNormal);
    	localPoint.set(cp.localPoint);
    	normal.set(cp.normal);
    	normalMass.set(cp.normalMass);
    	K.set(cp.K);
    	bodyA = cp.bodyA;
    	bodyB = cp.bodyB;
    	type = cp.type;
    	radius = cp.radius;
    	friction = cp.friction;
    	restitution = cp.restitution;
    	manifold = cp.manifold; // djm: not copy here
    	for(int i=0; i<cp.pointCount; i++){
    		points[i].set(cp.points[i]);
    	}
    }
}
