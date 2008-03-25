package org.jbox2d.dynamics;

import org.jbox2d.collision.Shape;

/// Implement this class to provide collision filtering. In other words, you can implement
/// this class if you want finer control over contact creation.
public interface ContactFilter {
	// Default contact filter, using groupIndex, maskBits and categoryBits as detailed
	// in Box2d manual.
	public static final ContactFilter DEFAULT_FILTER = new DefaultContactFilter();

	/// Return true if contact calculations should be performed between these two shapes.
	/// @warning for performance reasons this is only called when the AABBs begin to overlap.
	public boolean shouldCollide(Shape shape1, Shape shape2);
}
