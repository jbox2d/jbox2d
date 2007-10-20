package dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import dynamics.contacts.Contact;
import collision.Manifold;
import collision.*;

class PolyAndCircleContact extends Contact implements ContactCreator {

    Manifold m_manifold;

    public PolyAndCircleContact(Shape s1, Shape s2) {
        super(s1, s2);
        assert(m_shape1.m_type == ShapeType.POLY_SHAPE);
        assert(m_shape2.m_type == ShapeType.CIRCLE_SHAPE);
        m_manifold = new Manifold();
        m_manifoldCount = 0;
        // These should not be necessary, manifold was
        // just created...
        //m_manifold.pointCount = 0;
        //m_manifold.points[0].normalImpulse = 0.0f;
        //m_manifold.points[0].tangentImpulse = 0.0f;
    }

    public PolyAndCircleContact() {
        super();
        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public Contact clone() {
        PolyAndCircleContact newC = new PolyAndCircleContact(this.m_shape1, this.m_shape2);
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
        return new PolyAndCircleContact(shape1,shape2);
    }
    

    @Override
    public List<Manifold> GetManifolds() {
        List<Manifold> ret = new ArrayList<Manifold>(1);
        if (m_manifold != null) {
            ret.add(m_manifold);
        }
        return ret;
    }
    
    public void Evaluate() {
        //System.out.println("PolyAndCircleContact.Evaluate()");
        CollideCircle.b2CollidePolyAndCircle(m_manifold,(PolyShape)m_shape1,(CircleShape)m_shape2);
        
        if (m_manifold.pointCount > 0) {
            m_manifoldCount = 1;
        } else {
            m_manifoldCount = 0;
        }
    }
}