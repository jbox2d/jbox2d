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

import org.jbox2d.common.Vec2;

//Updated to rev 139 of b2Collision.h

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
	public final AABB set(final AABB aabb){
		lowerBound.set( aabb.lowerBound);
		upperBound.set( aabb.upperBound);
		return this;
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

	/** Check if AABBs overlap. djm optimized */
	public final boolean testOverlap(final AABB box) {
		final float d1x = box.lowerBound.x - upperBound.x;
		final float d1y = box.lowerBound.y - upperBound.y;
		final float d2x = lowerBound.x - box.upperBound.x;
		final float d2y = lowerBound.y - box.upperBound.y;

		// Vec2 d1 = box.lowerBound.sub(upperBound);
		//Vec2 d2 = lowerBound.sub(box.upperBound);

		if (d1x > 0.0f || d1y > 0.0f || d2x > 0.0f || d2y > 0.0f) {
			return false;
		}

		return true;
	}

	@Override
	public final String toString() {
		final String s = ""+lowerBound+" -> "+upperBound;
		return s;
	}
}
