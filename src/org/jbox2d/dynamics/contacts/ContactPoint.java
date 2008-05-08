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

package org.jbox2d.dynamics.contacts;

import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;

// Updated o rev 139 of b2Contact.h
public class ContactPoint {
		/** The first shape */
		public Shape shape1;		
		/** The second shape */
		public Shape shape2;		
		/** Position in world coordinates */
		public Vec2 position;
		/** Velocity of point on body2 relative to point on body1 (pre-solver) */
		public Vec2 velocity;
		/** Points from shape1 to shape2 */
		public Vec2 normal;
		/** The separation is negative when shapes are touching */
		public float separation;
		/** The combined friction coefficient */
		public float friction; 
		/** The combined restitution coefficient */
		public float restitution;
		/** The contact id identifies the features in contact */
		public ContactID id;
		
		public ContactPoint() {
			position = new Vec2();
			normal = new Vec2();
			id = new ContactID();
		}
}
