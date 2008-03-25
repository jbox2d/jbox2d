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
