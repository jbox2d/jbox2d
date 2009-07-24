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

import org.jbox2d.collision.structs.RayCastInput;
import org.jbox2d.collision.structs.RayCastOutput;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

//Updated to rev 139->218 of Collision.h

/** An axis-aligned bounding box. */
public class AABB {
	/** Bottom left vertex of bounding box. */
	public final Vec2 lowerBound;
	/** Top right vertex of bounding box. */
	public final Vec2 upperBound;

	/**
	 * Creates the default object, with vertices at 0,0 and 0,0.
	 */
	public AABB() {
		lowerBound = new Vec2();
		upperBound = new Vec2();
	}

	/**
	 * Copies from the given object
	 * @param copy the object to copy from
	 */
	public AABB(final AABB copy) {
		this(copy.lowerBound, copy.upperBound);
	}

	/**
	 * Creates an AABB object using the given bounding
	 * vertices.
	 * @param lowerVertex the bottom left vertex of the bounding box
	 * @param maxVertex the top right vertex of the bounding box
	 */
	public AABB(final Vec2 lowerVertex, final Vec2 upperVertex) {
		this.lowerBound = lowerVertex.clone(); // clone to be safe
		this.upperBound = upperVertex.clone();
	}


	/**
	 * Sets this object from the given object
	 * @param aabb the object to copy from
	 */
	public final void set(final AABB aabb){
		lowerBound.set( aabb.lowerBound);
		upperBound.set( aabb.upperBound);
	}

	/** Verify that the bounds are sorted*/
	public final boolean isValid() {
		final float dx = upperBound.x - lowerBound.x;
		final float dy = upperBound.y - lowerBound.y;
		if(!(dx >= 0.0f && dy >= 0)){
			return false;
		}
		return lowerBound.isValid() && upperBound.isValid();
	}

	/**
	 * Get the center of the AABB
	 * @return
	 */
	public final Vec2 getCenter(){
		final Vec2 center = new Vec2(lowerBound);
		center.addLocal(upperBound);
		center.mulLocal( .5f);
		return center;
	}

	public final void getCenterToOut(final Vec2 out){
		out.set( lowerBound);
		out.addLocal( upperBound);
		out.mulLocal(.5f);
	}
	/**
	 * Get the extents of the AABB (half-widths).
	 * @return
	 */
	public final Vec2 getExtents(){
		final Vec2 center = new Vec2(upperBound);
		center.subLocal( lowerBound);
		center.mulLocal( .5f);
		return center;
	}

	public final void getExtentsToOut(final Vec2 out){
		out.set(upperBound);
		out.subLocal( lowerBound);
		out.mulLocal( .5f);
	}

	/**
	 * Combine two AABBs into this one.
	 * @param aabb1
	 * @param aab
	 */
	public final void combine(final AABB aabb1, final AABB aab){
		Vec2.minToOut(aabb1.lowerBound, aab.lowerBound, lowerBound);
		Vec2.maxToOut(aabb1.upperBound, aab.upperBound, upperBound);
	}


	/**
	 * Does this aabb contain the provided AABB.
	 * @return
	 */
	public final boolean contains(final AABB aabb){
		/*boolean result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;*/
		// djm: faster using ors, as if one is true we leave the logic early
		return !(lowerBound.x > aabb.lowerBound.x ||
				lowerBound.y > aabb.lowerBound.y ||
				aabb.upperBound.x > upperBound.x ||
				aabb.upperBound.y > upperBound.y);
	}

	// djm pooled
	private static final Vec2 d = new Vec2();
	private static final Vec2 absD = new Vec2();
	private static final Vec2 normal = new Vec2();
	/**
	 * From Real-time Collision Detection, p179.
	 * @param output
	 * @param input
	 */
	public final void RayCast(final RayCastOutput output, final RayCastInput input){
		float tmin = Float.MIN_VALUE;
		float tmax = Float.MAX_VALUE;

		output.hit = false;

		final Vec2 p = input.p1;
		d.set(input.p2).subLocal(input.p1);
		Vec2.absToOut( d, absD);


		if (absD.x < Settings.EPSILON){
			// Parallel.
			if (p.x < lowerBound.x || upperBound.x < p.x){
				return;
			}
		}
		else{
			final float inv_d = 1.0f / d.x;
			float t1 = (lowerBound.x - p.x) * inv_d;
			float t2 = (upperBound.x - p.x) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if (t1 > t2){
				final float temp = t1;
				t1 = t2;
				t2 = temp;
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin){
				normal.setZero();
				normal.x = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = MathUtils.min(tmax, t2);

			if (tmin > tmax){
				return;
			}
		}

		if (absD.y < Settings.EPSILON){
			// Parallel.
			if (p.y < lowerBound.y || upperBound.y < p.y){
				return;
			}
		}
		else{
			final float inv_d = 1.0f / d.y;
			float t1 = (lowerBound.y - p.y) * inv_d;
			float t2 = (upperBound.y - p.y) * inv_d;

			// Sign of the normal vector.
			float s = -1.0f;

			if (t1 > t2){
				final float temp = t1;
				t1 = t2;
				t2 = temp;
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin){
				normal.setZero();
				normal.y = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = MathUtils.min(tmax, t2);

			if (tmin > tmax){
				return;
			}
		}

		// Does the ray start inside the box?
		// Does the ray intersect beyond the max fraction?
		if (tmin < 0.0f || input.maxFraction < tmin){
			return;
		}

		// Intersection.
		output.fraction = tmin;
		output.normal.set(normal);
		output.hit = true;
	}

	public static final boolean testOverlap(final AABB a, final AABB b){
		if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
			return false;
		}

		if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f) {
			return false;
		}

		return true;
	}

	@Override
	public final String toString() {
		final String s = ""+lowerBound+" . "+upperBound;
		return s;
	}
}
