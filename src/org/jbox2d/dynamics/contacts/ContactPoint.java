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

public class ContactPoint {
		public Shape shape1;		///< the first shape
		public Shape shape2;		///< the second shape
		public Vec2 position;		///< position in world coordinates
		public Vec2 normal;			///< points from shape1 to shape2
		public float separation;		///< the separation is negative when shapes are touching
		public float normalForce;	///< the signed magnitude of the normal force
		public float tangentForce;	///< the signed magnitude of the tangent force
		public ContactID id;			///< the contact id identifies the features in contact
		
		public ContactPoint() {
			position = new Vec2();
			normal = new Vec2();
		}
}
