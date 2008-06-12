package org.jbox2d.p5;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;

/**
 * A class to hold static methods for applying forces to bodies / shapes /
 * objects in various ways. If you find yourself duplicating code to do things
 * like this, then add it here!
 * 
 * @author Greg
 * 
 */
public class ForceUtils
{

	/** Pushes a body towards the specified location. */
	public void pushTowards(Body b, Vec2 worldTarget, double force)
	{
		Vec2 bodyVec = b.getWorldCenter();
		// First find the vector going from this body to the specified point
		worldTarget.subLocal(bodyVec);
		// Then, scale the vector to the specified force
		worldTarget.normalize();
		worldTarget.mulLocal((float) force);
		// Now apply it to the body's center of mass.
		b.applyForce(worldTarget, bodyVec);
	}

	/** Pushes a body in the specified direction. */
	public void push(Body b, Vec2 dir, double normalizedForce)
	{
		Vec2 bodyVec = b.getWorldCenter();
		dir.normalize();
		if (normalizedForce > 0)
		{
			dir.mulLocal((float) normalizedForce);
		}
		b.applyForce(dir, bodyVec);
	}

}
