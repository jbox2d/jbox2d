package org.jbox2d.dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.CircleShape;
import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.EdgeShape;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.ContactListener;

public class EdgeAndCircleContact extends Contact implements ContactCreateFcn {
	Manifold m_manifold;
	
	public EdgeAndCircleContact() {
		// TODO Auto-generated constructor stub
		super();
		m_manifold = new Manifold();
		m_manifoldCount = 0;
	}

	public EdgeAndCircleContact(Shape s1, Shape s2) {
		super(s1, s2);
		assert(m_shape1.getType() == ShapeType.EDGE_SHAPE);
		assert(m_shape2.getType() == ShapeType.CIRCLE_SHAPE);
		m_manifold = new Manifold();
		m_manifoldCount = 0;
	}

	@Override
	public Contact clone() {
		assert false: "Not yet implemented.";
    	return this;
	}
	
	public static void Destroy(Contact contact) {
        ((EdgeAndCircleContact) contact).destructor();
    }
	
	public void destructor() {
		
	}

	@Override
	public void evaluate(ContactListener listener) {
		Body b1 = m_shape1.getBody();
		Body b2 = m_shape2.getBody();

		Manifold m0 = new Manifold(m_manifold);
        for (int k = 0; k < m_manifold.pointCount; k++) {
            m0.points[k] = new ManifoldPoint(m_manifold.points[k]);
        }
        m0.pointCount = m_manifold.pointCount;

		CollideEdgeAndCircle(m_manifold, (EdgeShape)m_shape1, b1.getXForm(), (CircleShape)m_shape2, b2.getXForm());

		ContactPoint cp = new ContactPoint();
		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		//TODO
		cp.friction = m_friction;
		cp.restitution = m_restitution;
		//cp.friction = b2MixFriction(m_shape1->GetFriction(), m_shape2->GetFriction());
		//cp.restitution = b2MixRestitution(m_shape1->GetRestitution(), m_shape2->GetRestitution());

		if (m_manifold.pointCount > 0) {
			m_manifoldCount = 1;
			ManifoldPoint mp = m_manifold.points[0];

			if (m0.pointCount == 0) {
				mp.normalImpulse = 0.0f;
				mp.tangentImpulse = 0.0f;

				if (listener != null) {
					cp.position = b1.getWorldPoint(mp.localPoint1);
					Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
					Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity = v2.sub(v1);
					cp.normal = m_manifold.normal.clone();
					cp.separation = mp.separation;
					cp.id = new ContactID(mp.id);
					listener.add(cp);
				}
			} else {
				ManifoldPoint mp0 = m0.points[0];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;

				if (listener != null) {
					cp.position = b1.getWorldPoint(mp.localPoint1);
					Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
					Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
					cp.velocity = v2.sub(v1);
					cp.normal = m_manifold.normal.clone();
					cp.separation = mp.separation;
					cp.id = new ContactID(mp.id);
					listener.persist(cp);
				}
			}
		} else {
			m_manifoldCount = 0;
			if (m0.pointCount > 0 && (listener != null)) {
				ManifoldPoint mp0 = m0.points[0];
				cp.position = b1.getWorldPoint(mp0.localPoint1);
				Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp0.localPoint1);
				Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp0.localPoint2);
				cp.velocity = v2.sub(v1);
				cp.normal = m0.normal.clone();
				cp.separation = mp0.separation;
				cp.id = new ContactID(mp0.id);
				listener.remove(cp);
			}
		}

	}
	
	public void CollideEdgeAndCircle(Manifold manifold,
			final EdgeShape edge, final XForm xf1,
			final CircleShape circle, final XForm xf2) {
		manifold.pointCount = 0;
		Vec2 d = new Vec2();
		Vec2 c = XForm.mul(xf2, circle.getLocalPosition());
		Vec2 cLocal = XForm.mulT(xf1, c);
		Vec2 n = edge.getNormalVector();
		Vec2 v1 = edge.getVertex1();
		Vec2 v2 = edge.getVertex2();
		float radius = circle.getRadius();
		float separation;
		
		float dirDist = Vec2.dot((cLocal.sub(v1)), edge.getDirectionVector());
		if (dirDist <= 0) {
			d = cLocal.sub(v1);
			if (Vec2.dot(d, edge.getCorner1Vector()) < 0) {
				return;
			}
			d = c.sub(XForm.mul(xf1, v1));
		} else if (dirDist >= edge.getLength()) {
			d = cLocal.sub(v2);
			if (Vec2.dot(d, edge.getCorner2Vector()) > 0) {
				return;
			}
			d = c.sub(XForm.mul(xf1, v2));
		} else {
			separation = Vec2.dot(cLocal.sub(v1), n);
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;
			manifold.normal = Mat22.mul(xf1.R, n);
			manifold.pointCount = 1;
			manifold.points[0].id.zero();// key = 0;
			manifold.points[0].separation = separation;
			c.subLocal(manifold.normal.mul(radius));
			manifold.points[0].localPoint1 = XForm.mulT(xf1, c);
			manifold.points[0].localPoint2 = XForm.mulT(xf2, c);
			return;
		}
		
		float distSqr = Vec2.dot(d,d);
		if (distSqr > radius * radius) {
			return;
		}
		
		if (distSqr < Settings.EPSILON) {
			separation = -radius;
			manifold.normal = Mat22.mul(xf1.R, n);
		} else {
			separation = d.normalize() - radius;
			manifold.normal.set(d);
		}
		
		manifold.pointCount = 1;
		manifold.points[0].id.zero();//key = 0;
		manifold.points[0].separation = separation;
		c.subLocal(manifold.normal.mul(radius));
		manifold.points[0].localPoint1 = XForm.mulT(xf1, c);
		manifold.points[0].localPoint2 = XForm.mulT(xf2, c);

	}

	@Override
	public List<Manifold> getManifolds() {
		List<Manifold> ret = new ArrayList<Manifold>(1);
        if (m_manifold != null) {
            ret.add(m_manifold);
        }

        return ret;
	}

	public Contact create(Shape s1, Shape s2) {
		
		return new EdgeAndCircleContact(s1,s2);
	}

}
