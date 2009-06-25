package org.jbox2d.collision.structs;

import org.jbox2d.common.Vec2;

/**
 * Ray-cast output data.
 */
public class RayCastOutput{
	public final Vec2 normal;
	public float fraction;
	public boolean hit;

	public RayCastOutput(){
		normal = new Vec2();
		fraction = 0;
		hit = false;
	}

	public void set(final RayCastOutput rco){
		normal.set(rco.normal);
		fraction = rco.fraction;
		hit = rco.hit;
	}
};