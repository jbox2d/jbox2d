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

import collision.CollidePoly;
import collision.ContactID;
import collision.ContactPoint;
import collision.Manifold;
import collision.PolyShape;
import collision.Shape;
import collision.ShapeType;

public class PolyContact extends Contact implements ContactCreateFcn {

    Manifold m_manifold;

    public PolyContact(Shape s1, Shape s2) {
        super(s1, s2);
        assert (m_shape1.m_type == ShapeType.POLY_SHAPE);
        assert (m_shape2.m_type == ShapeType.POLY_SHAPE);

        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public PolyContact() {
        super();
        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public Contact clone() {
        PolyContact newC = new PolyContact(this.m_shape1, this.m_shape2);
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

    @Override
    public List<Manifold> GetManifolds() {
        // System.out.println("PolyContact.GetManifolds()");
        List<Manifold> ret = new ArrayList<Manifold>(1);
        if (m_manifold != null) {
            ret.add(m_manifold);
        }

        return ret;
    }

    public Contact create(Shape shape1, Shape shape2) {
        return new PolyContact(shape1, shape2);
    }

    @Override
    public void evaluate() {
        // Manifold m0 = m_manifold;
        Manifold m0 = new Manifold(m_manifold);
        for (int k = 0; k < m_manifold.pointCount; k++) {
            m0.points[k] = new ContactPoint(m_manifold.points[k]);
            m0.points[k].normalImpulse = m_manifold.points[k].normalImpulse;
            m0.points[k].tangentImpulse = m_manifold.points[k].tangentImpulse;
            //m0.points[k].id.key = m_manifold.points[k].id.key;
            m0.points[k].id.features.set(m_manifold.points[k].id.features);
            //System.out.println(m_manifold.points[k].id.key);
        }
        m0.pointCount = m_manifold.pointCount;

        CollidePoly.collidePoly(m_manifold, (PolyShape) m_shape1,
                (PolyShape) m_shape2, false);

        // Match contact ids to facilitate warm starting.
        // Watch out (Java note):
        // m_manifold.pointCount != m_manifold.points.length!!!
        if (m_manifold.pointCount > 0) {
            // Body b1 = m_shape1.m_body;
            // Body b2 = m_shape2.m_body;

            boolean match[] = new boolean[] { false, false };

            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (int i = 0; i < m_manifold.points.length; ++i) {
                ContactPoint cp = m_manifold.points[i];
                cp.normalImpulse = 0.0f;
                cp.tangentImpulse = 0.0f;
                ContactID id = cp.id;

                for (int j = 0; j < m0.points.length; ++j) {
                    if (match[j] == true) {
                        continue;
                    }

                    ContactPoint cp0 = m0.points[j];
                    ContactID id0 = new ContactID(cp0.id);

                    if (id0.features.isEqual(id.features)){
                    //if (id0.key == id.key) {
                        match[j] = true;
                        m_manifold.points[i].normalImpulse = m0.points[j].normalImpulse;
                        m_manifold.points[i].tangentImpulse = m0.points[j].tangentImpulse;
                        break;
                    }
                }
            }
            m_manifoldCount = 1;
        }
        else {
            m_manifoldCount = 0;
            // m_manifold = null; //Cleaner to actually store the count...
        }
    }
}
