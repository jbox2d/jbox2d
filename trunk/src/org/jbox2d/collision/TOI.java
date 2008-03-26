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

import org.jbox2d.common.*;

//updated to rev 108 of b2TimeOfImpact.cpp

public class TOI {
	// This algorithm uses conservative advancement to compute the time of
	// impact (TOI) of two shapes.
	// Refs: Bullet, Young Kim
	public static float timeOfImpact(Shape shape1, Sweep sweep1,
						   			 Shape shape2, Sweep sweep2) {
		
		float r1 = shape1.getSweepRadius();
		float r2 = shape2.getSweepRadius();

		assert(sweep1.t0 == sweep2.t0);
		assert(1.0f - sweep1.t0 > Settings.EPSILON);

		float t0 = sweep1.t0;
		Vec2 v1 = sweep1.c.sub(sweep1.c0);
		Vec2 v2 = sweep2.c.sub(sweep2.c0);
		float omega1 = sweep1.a - sweep1.a0;
		float omega2 = sweep2.a - sweep2.a0;

		float alpha = 0.0f;

		Vec2 p1 = new Vec2();
		Vec2 p2 = new Vec2();
		final int k_maxIterations = 20;	// TODO_ERIN b2Settings
		int iter = 0;
		Vec2 normal = new Vec2(0.0f, 0.0f);
		float distance = 0.0f;
		float targetDistance = 0.0f;
		while(true){
			float t = (1.0f - alpha) * t0 + alpha;
			XForm xf1 = new XForm();
			XForm xf2 = new XForm();
			sweep1.getXForm(xf1, t);
			sweep2.getXForm(xf2, t);

			// Get the distance between shapes.
			distance = Distance.distance(p1, p2, shape1, xf1, shape2, xf2);
			//System.out.println(distance);
			
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

			normal = p2.sub(p1);
			normal.normalize();

			// Compute upper bound on remaining movement.
			float approachVelocityBound = Vec2.dot(normal, v1.sub(v2)) + Math.abs(omega1) * r1 + Math.abs(omega2) * r2;
			if (Math.abs(approachVelocityBound) < Settings.EPSILON) {
				alpha = 1.0f;
				break;
			}

			// Get the conservative time increment. Don't advance all the way.
			float dAlpha = (distance - targetDistance) / approachVelocityBound;
			//float32 dt = (distance - 0.5f * b2_linearSlop) / approachVelocityBound;
			float newAlpha = alpha + dAlpha;

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
