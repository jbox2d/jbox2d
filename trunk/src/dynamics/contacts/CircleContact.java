package dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import collision.Manifold;
import collision.Shape;
import collision.ShapeType;
import collision.CollideCircle;
import collision.CircleShape;

public class CircleContact extends Contact implements ContactCreator {

    Manifold m_manifold;

    public Contact create(Shape shape1, Shape shape2) {
        return new CircleContact(shape1, shape2);
    }

    public static void Destroy(Contact contact) {
        ((CircleContact) contact).Destructor();
    }

    public CircleContact clone() {
        return this;
    }

    public CircleContact() {
        super();
        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public CircleContact(Shape shape1, Shape shape2) {
        super(shape1, shape2);
        m_manifold = new Manifold();
        assert (m_shape1.m_type == ShapeType.CIRCLE_SHAPE);
        assert (m_shape2.m_type == ShapeType.CIRCLE_SHAPE);
        m_manifold.pointCount = 0;
        m_manifold.points[0].normalImpulse = 0.0f;
        m_manifold.points[0].tangentImpulse = 0.0f;
    }

    public void Destructor() {

    }

    public void evaluate() {
        CollideCircle.collideCircle(m_manifold, (CircleShape) m_shape1,
                (CircleShape) m_shape2);

        if (m_manifold.pointCount > 0) {
            m_manifoldCount = 1;
        }
        else {
            m_manifoldCount = 0;
        }
    }

    @Override
    public List<Manifold> GetManifolds() {
        List<Manifold> ret = new ArrayList<Manifold>(1);
        if (m_manifold != null) {
            ret.add(m_manifold);
        }

        return ret;
    }

}