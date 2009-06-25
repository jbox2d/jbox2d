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

//Updated to rev 56->108->139->218 of b2Collision.h

/**
 * A manifold for two touching convex shapes.
 * Box2D supports multiple types of contact:
 * <ul><li>clip point versus plane with radius</li>
 * <li>point versus point with radius (circles)</li></ul>
 * The local point usage depends on the manifold type:
 * <ul><li>e_circles: the local center of circleA</li>
 * <li>e_faceA: the center of faceA</li>
 * <li>e_faceB: the center of faceB</li></ul>
 * Similarly the local normal usage:
 * <ul><li>e_circles: not used</li>
 * <li>e_faceA: the normal on polygonA</li>
 * <li>e_faceB: the normal on polygonB</li></ul>
 * We store contacts in this way so that position correction can
 * account for movement, which is critical for continuous physics.
 * All contact scenarios must be expressed in one of these types.
 * This structure is stored across time steps, so we keep it small.
 */
public class Manifold {
	
	public enum Type{
		e_circles,
		e_faceA,
		e_faceB
	}
	
	/** The points of contact. */
    public final ManifoldPoint[] m_points;
    
    /** not use for Type::e_points */
    public final Vec2 m_localPlaneNormal;
    
    /** usage depends on manifold type */
    public final Vec2 m_localPoint;
    
    Type m_type;
    
    /** The number of manifold points. */
    public int m_pointCount;

    /**
     * creates a manifold with 0 points, with it's points array
     * full of instantiated ManifoldPoints.
     */
    public Manifold() {
        m_points = new ManifoldPoint[Settings.maxManifoldPoints];
        for (int i = 0; i < Settings.maxManifoldPoints; i++) {
            m_points[i] = new ManifoldPoint();
        }
        m_localPlaneNormal = new Vec2();
        m_localPoint = new Vec2();
        m_pointCount = 0;
    }

    /**
     * Creates this manifold as a copy of the other
     * @param other
     */
    public Manifold(Manifold other) {
        m_points = new ManifoldPoint[Settings.maxManifoldPoints];
        m_localPlaneNormal = other.m_localPlaneNormal.clone();
        m_localPoint = other.m_localPoint.clone();
        m_pointCount = other.m_pointCount;
        // djm: this is correct now
        for(int i=0; i < other.m_points.length; i++){
    		m_points[i] = new ManifoldPoint(other.m_points[i]);
    	}
    }
    
    // djm for object reusability
    /**
     * copies this manifold from the given one
     * @param cp manifold to copy from
     */
    public void set(Manifold cp){
    	// will this work?
    	//System.arraycopy(cp.points, 0, points, 0, cp.pointCount);
    	// just do this for now
    	for(int i=0; i<cp.m_pointCount; i++){
    		m_points[i].set(cp.m_points[i]);
    	}
    	
    	m_localPlaneNormal.set(cp.m_localPlaneNormal);
    	m_localPoint.set( cp.m_localPoint);
    	m_pointCount = cp.m_pointCount;
    }
}
