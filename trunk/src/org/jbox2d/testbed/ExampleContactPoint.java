package org.jbox2d.testbed;

import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Vec2;

/**
 * Holder for storing contact information.
 */
public class ExampleContactPoint {
	public Shape shape1;
	public Shape shape2;
	public Vec2 normal;
	public Vec2 position;
	public Vec2 velocity;
	ContactID id;
	public int state; // 0-add, 1-persist, 2-remove
}