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
	/** Vertex of bounding box. */
    public Vec2 lowerBound, upperBound;
    
    public String toString() {
    	String s = ""+lowerBound+" -> "+upperBound;
    	return s;
    }

    public AABB(Vec2 minVertex, Vec2 maxVertex) {
        this.lowerBound = minVertex.clone(); // clone to be safe
        this.upperBound = maxVertex.clone();
    }

    public AABB(AABB copy) {
        this(copy.lowerBound, copy.upperBound); // relies on cloning in AABB(Vec2, Vec2) constructor
    }

    public AABB() {
        lowerBound = new Vec2();
        upperBound = new Vec2();
    }
    
    // DMNOTE needed for the stack
    public void set(AABB aabb){
    	lowerBound.set( aabb.lowerBound);
    	upperBound.set( aabb.upperBound);
    }

    /** Verify that the bounds are sorted. DMNOTE optimized */
    public boolean isValid() {
        float dx = upperBound.x - lowerBound.x;
        float dy = upperBound.y - lowerBound.y;
    	if(!(dx >= 0.0f && dy >= 0)){
    		return false;
    	}
    	return lowerBound.isValid() && upperBound.isValid();
    }

    /** Check if AABBs overlap. DMNOTE optimized */
    public boolean testOverlap(AABB box) {
        float d1x = box.lowerBound.x - upperBound.x;
        float d1y = box.lowerBound.y - upperBound.y;
        float d2x = lowerBound.x - box.upperBound.x;
        float d2y = lowerBound.y - box.upperBound.y;

       // Vec2 d1 = box.lowerBound.sub(upperBound);
        //Vec2 d2 = lowerBound.sub(box.upperBound);

        if (d1x > 0.0f || d1y > 0.0f || d2x > 0.0f || d2y > 0.0f) {
            return false;
        }
        else {
            return true;
        }
    }
}
