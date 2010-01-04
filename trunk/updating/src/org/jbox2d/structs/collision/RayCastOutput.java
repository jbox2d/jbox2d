package org.jbox2d.structs.collision;

import org.jbox2d.common.Vec2;

/**
 * Ray-cast output data.
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