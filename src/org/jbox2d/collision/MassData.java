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

// Updated to rev 142 of b2Shape.h

/** This holds the mass data computed for a shape. */
public class MassData {
	/** The mass of the shape, usually in kilograms. */
    public float mass;
    /** The position of the shape's centroid relative to the shape's origin. */
    public Vec2 center;
    /** The rotational inertia of the shape. */
    public float I;

    public MassData() {
        mass = I = 0f;
        center = new Vec2();
    }
    
    public MassData(MassData md) {
    	mass = md.mass;
    	I = md.I;
    	center = md.center.clone();
    }
    
    /** Return a copy of this object. */
    public MassData clone() {
    	return new MassData(this);
    }
}
