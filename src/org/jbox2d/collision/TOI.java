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
