package org.jbox2d.testbed;

import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;

/**
 * Holder for storing contact information.
 */
public class ExampleContactPoint {
	public Shape shape1;
	public Shape shape2;
	public final Vec2 normal = new Vec2();
	public final Vec2 position = new Vec2();
	public final Vec2 velocity = new Vec2();
	public final ContactID id = new ContactID();
	public int state; // 0-add, 1-persist, 2-remove
}