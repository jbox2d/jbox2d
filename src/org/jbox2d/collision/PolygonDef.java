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

//Updated to rev 56 of b2Shape.cpp/.h
// -> rev 97 of b2PolygonShape.cpp/.h

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.common.*;

/**
 * Convex polygon. The vertices must be in CCW order for a right-handed
 * coordinate system with the z-axis coming out of the screen.
 * Add vertices using PolygonDef.add(Vec2),
 * and create the polygon shape using Body::createShape(ShapeDef).
 */
public class PolygonDef extends ShapeDef {

	/** 
	 * The polygon vertices in local coordinates.
	 * <BR><BR>
	 * Accessing this field is discouraged - it remains
	 * public for the moment, but that is likely to change.
	 * Please use addVertex(Vec2) and getVertexList/Array
	 * instead to add to or inspect the current vertices.
	 */
    public List<Vec2> vertices;

    public PolygonDef() {
        type = ShapeType.POLYGON_SHAPE;
        vertices = new ArrayList<Vec2>();
    }
    
    /** Add a vertex to the polygon. */
    public void addVertex(Vec2 v) {
    	vertices.add(v);
    }
    
    /** Removes all vertices. */
    public void clearVertices() {
    	vertices.clear();
    }
    
    /** Return the vertex list as an array. */
    public Vec2[] getVertexArray() {
    	return vertices.toArray(new Vec2[0]);
    }
    
    /** Return the vertex list as a List<Vec2>. */
    public List<Vec2> getVertexList() {
    	return vertices;
    }
    
    /**
     * Build vertices to represent an axis-aligned box.
	 * @param hx the half-width.
	 * @param hy the half-height.
	 */
    public void setAsBox(float hx, float hy) {
    	vertices.clear();
    	vertices.add(new Vec2(-hx, -hy));
    	vertices.add(new Vec2(hx, -hy));
    	vertices.add(new Vec2(hx, hy));
    	vertices.add(new Vec2(-hx, hy));
    }

    /**
     * Build vertices to represent an oriented box.
	 * @param hx the half-width.
	 * @param hy the half-height.
	 * @param center the center of the box in local coordinates.
	 * @param angle the rotation of the box in local coordinates.
	 */
    public void setAsBox(float hx, float hy, Vec2 center, float angle) {
    	setAsBox(hx, hy);
    	XForm xf = new XForm();
    	xf.position.set(center);
    	xf.R.set(angle);

    	for (int i = 0; i < vertices.size(); ++i) {
    		vertices.get(i).set(XForm.mul(xf, vertices.get(i)));
    	}
    }

    /** Return the number of vertices. */
    public int getVertexCount() {
        return vertices.size();
    }
}