package org.jbox2d.p5;


import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;

public class PhysicsUtils
{
	/**
	 * In case you want to do something to a body during each creation (such as add an Actor object),
	 * you can override the PhysicsUtils class and override this method.
	 * @return
	 */
	public static BodyDef newBodyDef()
	{
		BodyDef bd = new BodyDef();
		return bd;
	}

	/**
	 * Returns the clockwise angle from vector A to vector B.
	 * @param a
	 * @param b
	 * @return
	 */
	public static float angle(Vec2 a, Vec2 b)
	{
		//float lenA = a.length();
		//float lenB = b.length();
		
		//Done: fix this up to use atan2, which is better than acos for angle finding - the
		//problem is that acos will only return results between 0 and pi due to
		//the double valued-ness of the cosine on the circle; it can't distinguish between
		//positive and negative angles, in other words, so it returns the same result for
		//both.  atan2 goes all the way from -pi to +pi, so is almost always what you want.
		//float cosTheta = Vec2.dot(a, b) / (lenA*lenB);
		float theta = (float)(Math.atan2(b.y,b.x) - Math.atan2(a.y, a.x));
		return theta;
	}
	
	/**
	 * Returns the clockwise angle of vector A, relative to the horizontal.
	 * @param a
	 * @return
	 */
	public static float angle(Vec2 a)
	{
		return angle(new Vec2(1,0),a);
	}

}
