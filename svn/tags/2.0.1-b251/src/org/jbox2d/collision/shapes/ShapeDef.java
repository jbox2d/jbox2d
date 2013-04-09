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

package org.jbox2d.collision.shapes;

import org.jbox2d.collision.FilterData;

// Updated to rev 139 of b2Shape.h

/**
 * Superclass for shape definitions.
 * You should usually use CircleDef or
 * PolygonDef to define concrete shapes.
 */
public class ShapeDef {
	/** Holds the shape type for down-casting. */
	public ShapeType type;

	/** Use this to store application specify shape data. */
	public Object userData;

	/** The shape's friction coefficient, usually in the range [0,1]. */
	public float friction;

	/** The shape's restitution (elasticity) usually in the range [0,1]. */
	public float restitution;

	/** The shape's density, usually in kg/m^2. */
	public float density;

	/** Contact filtering data. */
	public FilterData filter;

	/**
	 * A sensor shape collects contact information but never generates a collision
	 * response.
	 */
	public boolean isSensor;

	public ShapeDef() {
		type = ShapeType.UNKNOWN_SHAPE;
		userData = null;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		filter = new FilterData();
		filter.categoryBits = 0x0001;
		filter.maskBits = 0xFFFF;
		filter.groupIndex = 0;
		isSensor = false;
	}
}