package dynamics;

import collision.*;

// Implement this class to provide collision filtering. In other words, you can implement
// this class if you want finer control over contact creation.
public class CollisionFilter {
    static public CollisionFilter DEFAULT_FILTER = new CollisionFilter();
    // Return true if contact calculations should be performed between these two shapes.
    public boolean shouldCollide(Shape shape1, Shape shape2) {
        if (shape1.m_groupIndex == shape2.m_groupIndex && shape1.m_groupIndex != 0) {
            return shape1.m_groupIndex > 0;
        }

        boolean collide = (shape1.m_maskBits & shape2.m_categoryBits) != 0 && (shape1.m_categoryBits & shape2.m_maskBits) != 0;
        return collide;
    }
}