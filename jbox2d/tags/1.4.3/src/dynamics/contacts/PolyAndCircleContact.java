/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.gphysics.com
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
package dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import collision.CircleShape;
import collision.CollideCircle;
import collision.Manifold;
import collision.PolyShape;
import collision.Shape;
import collision.ShapeType;

class PolyAndCircleContact extends Contact implements ContactCreateFcn {

    Manifold m_manifold;

    public PolyAndCircleContact(Shape s1, Shape s2) {
        super(s1, s2);
        assert (m_shape1.m_type == ShapeType.POLY_SHAPE);
        assert (m_shape2.m_type == ShapeType.CIRCLE_SHAPE);
        m_manifold = new Manifold();
        m_manifoldCount = 0;
        // These should not be necessary, manifold was
        // just created...
        // m_manifold.pointCount = 0;
        // m_manifold.points[0].normalImpulse = 0.0f;
        // m_manifold.points[0].tangentImpulse = 0.0f;
    }

    public PolyAndCircleContact() {
        super();
        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public Contact clone() {
        PolyAndCircleContact newC = new PolyAndCircleContact(this.m_shape1,
                this.m_shape2);
        newC.m_manifold = new Manifold(this.m_manifold);
        newC.m_manifoldCount = this.m_manifoldCount;
        // The parent world.
        newC.m_world = this.m_world;

        // World pool and list pointers.
        newC.m_prev = this.m_prev;
        newC.m_next = this.m_next;

        // Nodes for connecting bodies.
        newC.m_node1 = this.m_node1;
        newC.m_node2 = this.m_node2;

        // Combined friction
        newC.m_friction = this.m_friction;
        newC.m_restitution = this.m_restitution;

        newC.m_flags = this.m_flags;
        return newC;
    }

    public Contact create(Shape shape1, Shape shape2) {
        return new PolyAndCircleContact(shape1, shape2);
    }

    @Override
    public List<Manifold> GetManifolds() {
        List<Manifold> ret = new ArrayList<Manifold>(1);
        if (m_manifold != null) {
            ret.add(m_manifold);
        }
        return ret;
    }

    public void evaluate() {
        // System.out.println("PolyAndCircleContact.Evaluate()");
        CollideCircle.collidePolyAndCircle(m_manifold, (PolyShape) m_shape1,
                (CircleShape) m_shape2, false);

        if (m_manifold.pointCount > 0) {
            m_manifoldCount = 1;
        }
        else {
            m_manifoldCount = 0;
        }
    }
}