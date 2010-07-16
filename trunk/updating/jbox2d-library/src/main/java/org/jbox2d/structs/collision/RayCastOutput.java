package org.jbox2d.structs.collision;

import org.jbox2d.common.Vec2;

// updated to rev 100
/**
 * Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
 * come from b2RayCastInput.
 */
public class RayCastOutput{
	public final Vec2 normal;
	public float fraction;

	public RayCastOutput(){
		normal = new Vec2();
		fraction = 0;
	}

	public void set(final RayCastOutput rco){
		normal.set(rco.normal);
		fraction = rco.fraction;
	}
};