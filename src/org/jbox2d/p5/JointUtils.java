package org.jbox2d.p5;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.DistanceJoint;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

public class JointUtils
{
	/**
	 * Creates a distance joint between bodies that touch the given points,
	 * anchored at those points. Returns null if there are not two different
	 * bodies at the points given. Behavior is undetermined if more than one
	 * body touches a point.
	 * 
	 * @param w
	 *            World to add joint to and read bodies from
	 * @param pA
	 *            First point to attach to
	 * @param pB
	 *            Second point to attach to
	 * @return Joint created, or null if impossible to create the joint
	 */
	public static DistanceJoint createDistanceJoint(World w, Vec2 pA, Vec2 pB)
	{
		Body[] bodies = getTwoDistinctBodies(w, pA, pB);
		if (bodies == null || bodies.length != 2)
			return null;
		DistanceJointDef jd = new DistanceJointDef();
		jd.initialize(bodies[0], bodies[1], pA, pB);
		return (DistanceJoint) w.createJoint(jd);
	}

	/**
	 * Creates and returns a simple distance joint between two bodies, based on
	 * their world body positions (NOT their centers of mass).
	 * 
	 * @param a
	 * @param b
	 * @return a new DistanceJoint
	 */
	public static DistanceJoint createDistanceJoint(Body a, Body b)
	{
		DistanceJointDef jd = new DistanceJointDef();
		jd.body1 = a;
		jd.body2 = b;
		/*
		 * Auto-length based on world body positions.
		 */
		float length = a.getPosition().subLocal(b.getPosition()).length();
		jd.length = length;

		/*
		 * Note to self: Local anchors are already set to (0,0) by the constructor.
		 */
		DistanceJoint j = (DistanceJoint) a.getWorld().createJoint(jd);
		return j;
	}

	/**
	 * Creates a RevoluteJoint between two bodies that rotates around a given
	 * point in World coordinates.
	 * 
	 * @param a
	 * @param b
	 * @param worldCenter
	 * @return
	 */
	public static RevoluteJoint createRevoluteJoint(Body a, Body b, Vec2 worldCenter)
	{
		RevoluteJointDef jd = new RevoluteJointDef();
		jd.body1 = a;
		jd.body2 = b;

		jd.localAnchor1 = a.getLocalPoint(worldCenter);
		jd.localAnchor2 = b.getLocalPoint(worldCenter);

		RevoluteJoint j = (RevoluteJoint) a.getWorld().createJoint(jd);
		return j;
	}

	/**
	 * <p>
	 * Finds a pair of non-identical bodies that have shapes touching points pA
	 * and pB, loaded resp. into the 0th and 1st elements of the returned array.
	 * Returns null array if no such pair exists.
	 * </p>
	 * <p>
	 * This method is useful for creating joints based on location, where the
	 * two bodies in a joint must be distinct.
	 * </p>
	 * 
	 * @param w
	 * @param pA
	 * @param pB
	 * @return a Body array containing two distinct bodies, or NULL if no
	 *         distinct bodies exist at the given points.
	 */
	private static Body[] getTwoDistinctBodies(World w, Vec2 pA, Vec2 pB)
	{
		// Package accessible until proven to need more
		AABB aabb1 = new AABB(new Vec2(pA.x - .001f, pA.y - .001f), new Vec2(pA.x + .001f, pA.y + .001f));
		AABB aabb2 = new AABB(new Vec2(pB.x - .001f, pB.y - .001f), new Vec2(pB.x + .001f, pB.y + .001f));
		int maxCount = 10;
		Shape[] shapes1 = w.query(aabb1, maxCount);
		if (shapes1[0] == null)
			return null;
		Shape[] shapes2 = w.query(aabb2, maxCount);
		if (shapes2[0] == null)
			return null;
		boolean found = false;
		int i = 0;
		int j = 0;
		Body body1 = null, body2 = null;
		// Try to find a pair of bodies that are not equal
		// and overlap the appropriate points - this whole
		// mess is just to catch the rare edge case where a single
		// body overlaps both points, but another body is there, too.
		// Which ones are selected are up to fate...
		while (!found)
		{
			if (i >= shapes1.length || shapes1[i] == null)
				return null;
			if (shapes1[i].testPoint(shapes1[i].getBody().getXForm(), pA))
				body1 = shapes1[i++].getBody();
			for (j = 0; j < shapes2.length; ++j)
			{
				if (shapes2[j] == null)
					break;
				if (shapes2[j].testPoint(shapes2[j].getBody().getXForm(), pB))
					body2 = shapes2[j].getBody();
				if (body2 != body1)
				{
					found = true;
					break;
				}
			}
		}
		if (body1 == null || body2 == null)
			return null;
		Body[] bodies = new Body[] { body1, body2 };
		return bodies;
	}

}
