package org.jbox2d.collision.structs;

import org.jbox2d.common.Vec2;

/**
 * Ray-cast input data.
 */
public class RayCastInput{
	public final Vec2 p1, p2;
	public float maxFraction;

	public RayCastInput(){
		p1 = new Vec2();
		p2 = new Vec2();
		maxFraction = 0;
	}

	public void set(final RayCastInput rci){
		p1.set(rci.p1);
		p2.set(rci.p2);
		maxFraction = rci.maxFraction;
	}
}