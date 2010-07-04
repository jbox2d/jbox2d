package org.jbox2d.p5;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Steppable;
import org.jbox2d.dynamics.joints.Joint;

/**
 * Tracks the reaction force of a given joint, and destroys the joint if the reaction force is above
 * a given threshold AND is between the minimum and maximum allowable angles. This is useful for creating
 * mouse-controlled joints where the "release" should only happen in one direction, and when a certain
 * force threshold is reached.
 * @author Greg
 *
 */
public class JointBreaker implements Steppable
{
	Joint joint;
	float reactionThreshold;
	float angleLo;
	float angleHi;

	public JointBreaker(Joint joint, float reactionThreshold)
	{
		this.joint = joint;
		this.reactionThreshold = reactionThreshold;
	}

	public void step(float dt, int iterations)
	{
		if (joint != null)
		{
			Vec2 force = joint.getReactionForce();
			float forceAngle = PhysicsUtils.angle(force);
			float forceMag = force.length();
			
			if (forceMag > reactionThreshold)
			{
				/*
				 * First, check for direction.
				 */
				if (angleLo == angleHi || (forceAngle >= angleLo && forceAngle <= angleHi))
				{
					// Ok, this one's done with. Kill it.
					joint.getBody1().getWorld().destroyJoint(joint);
					joint = null;
					joint.getBody1().getWorld().unregisterPostStep(this);
				}
			}
		}
	}
	
	/**
	 * Sets the range of angles between which this jointbreaker will break the joint. Angles are in radians,
	 * relative to due east.
	 * @param lo
	 * @param hi
	 */
	public void setAngleLimits(float lo, float hi)
	{
		angleLo = lo;
		angleHi = hi;
	}
}
