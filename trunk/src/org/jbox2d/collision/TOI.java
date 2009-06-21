/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.collision;

import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Sweep;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

//updated to rev 142 of b2TimeOfImpact.cpp

/** Handles conservative advancement to compute time of impact between shapes. */
public class TOI {

	// These are used to avoid allocations on a hot path:
	private static final Vec2 p1 = new Vec2();
	private static final Vec2 p2 = new Vec2();
	private static final XForm xf1 = new XForm();
	private static final XForm xf2 = new XForm();

	// This algorithm uses conservative advancement to compute the time of
	// impact (TOI) of two shapes.
	// Refs: Bullet, Young Kim

	/**
	 * Compute the time when two shapes begin to touch or touch at a closer distance.
	 * <BR><BR><em>Warning</em>: the sweeps must have the same time interval.
	 * @return the fraction between [0,1] in which the shapes first touch.
	 * fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
	 */
	public static final float timeOfImpact(final Shape shape1, final Sweep sweep1,
	                                       final Shape shape2, final Sweep sweep2) {

		final float r1 = shape1.getSweepRadius();
		final float r2 = shape2.getSweepRadius();

		assert(sweep1.t0 == sweep2.t0);
		assert(1.0f - sweep1.t0 > Settings.EPSILON);

		final float t0 = sweep1.t0;
		// INLINED
		//Vec2 v1 = sweep1.c.sub(sweep1.c0);
		//Vec2 v2 = sweep2.c.sub(sweep2.c0);
		//Vec2 v = v1.sub(v2);
		final float vx = (sweep1.c.x - sweep1.c0.x) - (sweep2.c.x - sweep2.c0.x);
		final float vy = (sweep1.c.y - sweep1.c0.y) - (sweep2.c.y - sweep2.c0.y);

		final float omega1 = sweep1.a - sweep1.a0;
		final float omega2 = sweep2.a - sweep2.a0;

		float alpha = 0.0f;

		final int k_maxIterations = 20;	// TODO_ERIN b2Settings
		int iter = 0;
		float distance = 0.0f;
		float targetDistance = 0.0f;
		while(true){
			final float t = (1.0f - alpha) * t0 + alpha;
			sweep1.getXForm(TOI.xf1, t);
			sweep2.getXForm(TOI.xf2, t);

			// Get the distance between shapes.
			distance = Distance.distance(TOI.p1, TOI.p2, shape1, TOI.xf1, shape2, TOI.xf2);
			//System.out.println("Distance: "+distance + " alpha: "+alpha);

			if (iter == 0) {
				// Compute a reasonable target distance to give some breathing room
				// for conservative advancement.
				if (distance > 2.0f * Settings.toiSlop) {
					targetDistance = 1.5f * Settings.toiSlop;
				} else {
					targetDistance = Math.max(0.05f * Settings.toiSlop, distance - 0.5f * Settings.toiSlop);
				}
			}

			if (distance - targetDistance < 0.05f * Settings.toiSlop || iter == k_maxIterations) {
				//if (distance-targetDistance < 0) System.out.println("dist error: "+ (distance-targetDistance) + " toiSlop: "+Settings.toiSlop + " iter: "+iter);
				break;
			}

			// INLINED
			//normal = p2.sub(p1);
			//normal.normalize();
			float normalx = TOI.p2.x - TOI.p1.x;
			float normaly = TOI.p2.y - TOI.p1.y;
			final float lenSqrd = normalx * normalx + normaly * normaly;
			if (lenSqrd >= Settings.EPSILON*Settings.EPSILON) {
				final float length = (float) Math.sqrt(lenSqrd);
				final float invLength = 1.0f / length;
				normalx *= invLength;
				normaly *= invLength;
			}

			// Compute upper bound on remaining movement.
			// INLINED
			//float approachVelocityBound = Vec2.dot(normal, v) + Math.abs(omega1) * r1 + Math.abs(omega2) * r2;
			final float approachVelocityBound = (normalx * vx + normaly * vy) + Math.abs(omega1) * r1 + Math.abs(omega2) * r2;
			//System.out.println("avb: "+approachVelocityBound);
			//System.out.println("Normal" + normal);
			if (Math.abs(approachVelocityBound) < Settings.EPSILON) {
				alpha = 1.0f;
				break;
			}

			// Get the conservative time increment. Don't advance all the way.
			final float dAlpha = (distance - targetDistance) / approachVelocityBound;
			//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
			final float newAlpha = alpha + dAlpha;

			// The shapes may be moving apart or a safe distance apart.
			if (newAlpha < 0.0f || 1.0f < newAlpha) {
				alpha = 1.0f;
				break;
			}

			// Ensure significant advancement.
			if (newAlpha < (1.0f + 100.0f * Settings.EPSILON) * alpha) {
				break;
			}

			alpha = newAlpha;

			++iter;
		}

		return alpha;
	}
}
