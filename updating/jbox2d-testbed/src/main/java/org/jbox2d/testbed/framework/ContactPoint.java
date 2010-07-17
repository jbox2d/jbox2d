/**
 * Created at 2:12:15 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.structs.collision.PointState;

/**
 * @author Daniel Murphy
 */
public class ContactPoint {
	public Fixture fixtureA;
	public Fixture fixtureB;
	public final Vec2 normal = new Vec2();
	public final Vec2 position = new Vec2();
	public PointState state;
}
