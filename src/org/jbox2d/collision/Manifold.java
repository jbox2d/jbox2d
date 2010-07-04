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

import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;

//Updated to rev 56->108->139 of b2Collision.h

/** A manifold for two touching convex shapes. */
public class Manifold {
	/** The points of contact. */
    public final ManifoldPoint[] points;
    /** The shared unit normal vector. */
    public final Vec2 normal;
    /** The number of manifold points. */
    public int pointCount;

    /**
     * creates a manifold with 0 points, with it's points array
     * full of instantiated ManifoldPoints.
     */
    public Manifold() {
        points = new ManifoldPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            points[i] = new ManifoldPoint();
        }
        normal = new Vec2();
        pointCount = 0;
    }

    /**
     * Creates this manifold as a copy of the other
     * @param other
     */
    public Manifold(Manifold other) {
        points = new ManifoldPoint[Settings.maxManifoldPoints];
        normal = other.normal.clone();
        pointCount = other.pointCount;
        // djm this is correct now
        for(int i=0; i < other.points.length; i++){
    		points[i] = new ManifoldPoint(other.points[i]);
    	}
    }
    
    // djm for object reusability
    /**
     * copies this manifold from the given one
     * @param cp manifold to copy from
     */
    public Manifold set(Manifold cp){
    	// will this work?
    	//System.arraycopy(cp.points, 0, points, 0, cp.pointCount);
    	// just do this for now
    	for(int i=0; i<cp.pointCount; i++){
    		points[i].set(cp.points[i]);
    	}
    	
    	normal.set(cp.normal);
    	pointCount = cp.pointCount;
    	return this;
    }
}
