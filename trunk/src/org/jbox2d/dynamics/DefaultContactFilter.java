package org.jbox2d.dynamics;

import org.jbox2d.collision.Shape;

public class DefaultContactFilter implements ContactFilter {

	// Return true if contact calculations should be performed between these two shapes.
	// If you implement your own collision filter you may want to build from this implementation.
	public boolean shouldCollide(Shape shape1, Shape shape2) {

		if (shape1.m_groupIndex == shape2.m_groupIndex && shape1.m_groupIndex != 0) {
			return shape1.m_groupIndex > 0;
		}

		boolean collide = (shape1.m_maskBits & shape2.m_categoryBits) != 0 && (shape1.m_categoryBits & shape2.m_maskBits) != 0;
		return collide;
	}

}
