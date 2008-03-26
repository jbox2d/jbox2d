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

package org.jbox2d.dynamics;

//Updated to rev. 56->118 of b2Body.h

import org.jbox2d.collision.MassData;
import org.jbox2d.common.Vec2;


public class BodyDef {
	
	/// You can use this to initialized the mass properties of the body.
	/// If you prefer, you can set the mass properties after the shapes
	/// have been added using b2Body::SetMassFromShapes.
	public MassData massData;
	
	/// Use this to store application specific body data
    public Object userData;

    /// The world position of the body.  Avoid creating bodies at the origin
    /// since this can lead to many overlapping shapes.
    public Vec2 position;
    
    /// The world angle of the body in radians.
    public float angle;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	public float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	public float angularDamping;
	
	/// Set this flag to false if this body should never fall asleep.  Note that
	/// this increases CPU usage.
    public boolean allowSleep;

    /// Is this body initially sleeping?
    public boolean isSleeping;
    
    /// Should this body be prevented from rotating?  Useful for characters.
    public boolean fixedRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// static bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	public boolean isBullet;

	public BodyDef() {
		massData = new MassData();
        massData.center = new Vec2(0.0f, 0.0f);
        massData.mass = 0.0f;
        massData.I = 0.0f;
        userData = null;
        position = new Vec2(0.0f, 0.0f);
        angle = 0.0f;
        linearDamping = 0.0f;
        angularDamping = 0.0f;
        allowSleep = true;
        isSleeping = false;
        fixedRotation = false;
        isBullet = false;
    }

}
