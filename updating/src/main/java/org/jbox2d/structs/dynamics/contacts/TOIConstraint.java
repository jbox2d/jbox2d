/**
 * Created at 2:26:49 PM Jul 6, 2010
 */
package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.structs.collision.Manifold.ManifoldType;

// updated to rev 100
/**
 * @author daniel
 */
public class TOIConstraint {
	public final Vec2 localPoints[] = new Vec2[Settings.maxManifoldPoints];
	public final Vec2 localNormal = new Vec2();
	public final Vec2 localPoint = new Vec2();
	public ManifoldType type;
	public float radius;
	public int pointCount;
	public Body bodyA;
	public Body bodyB;
	
	public TOIConstraint(){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i] = new Vec2();
		}
	}
	
	public TOIConstraint(TOIConstraint argToClone){
		this();
		set(argToClone);
	}
	
	public void set(TOIConstraint argOther){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i].set(argOther.localPoints[i]);
		}
		localNormal.set(argOther.localNormal);
		localPoint.set(argOther.localPoint);
		type = argOther.type;
		radius = argOther.radius;
		pointCount = argOther.pointCount;
		bodyA = argOther.bodyA;
		bodyB = argOther.bodyB;
	}
}
