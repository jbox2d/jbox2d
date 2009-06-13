package org.jbox2d.dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.CircleShape;
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

	// djm pooled
	private Manifold m0 = new Manifold();
	private Vec2 v1 = new Vec2();
	@Override
	public void evaluate(ContactListener listener) {
		Body b1 = m_shape1.getBody();
		Body b2 = m_shape2.getBody();

		m0.set(m_manifold);

		CollideEdgeAndCircle(m_manifold, (EdgeShape)m_shape1, b1.getMemberXForm(), (CircleShape)m_shape2, b2.getMemberXForm());

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
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
    				b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
    				// djm cp.velocity isn't instantiated in the constructor,
    				// so we just create it here
    				cp.velocity = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
    				//cp.velocity = v2.sub(v1);
    				cp.velocity.subLocal(v1);
    				
					cp.normal.set(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.set(mp.id);
					listener.add(cp);
				}
			} else {
				ManifoldPoint mp0 = m0.points[0];
				mp.normalImpulse = mp0.normalImpulse;
				mp.tangentImpulse = mp0.tangentImpulse;

				if (listener != null) {
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					//cp.position = b1.getWorldLocation(mp.localPoint1);
					//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
    				b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
    				// djm cp.velocity isn't instantiated in the constructor,
    				// so we just create it here
    				cp.velocity = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
    				//cp.velocity = v2.sub(v1);
    				cp.velocity.subLocal(v1);
    				
					cp.normal.set(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.set(mp.id);
					listener.persist(cp);
				}
			}
		} else {
			m_manifoldCount = 0;
			if (m0.pointCount > 0 && (listener != null)) {
				ManifoldPoint mp0 = m0.points[0];
				b1.getWorldLocationToOut(mp0.localPoint1, cp.position);
				//cp.position = b1.getWorldLocation(mp0.localPoint1);
				//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
				b1.getLinearVelocityFromLocalPointToOut(mp0.localPoint1, v1);
				// djm cp.velocity isn't instantiated in the constructor,
				// so we just create it here
				cp.velocity = b2.getLinearVelocityFromLocalPoint(mp0.localPoint2);
				//cp.velocity = v2.sub(v1);
				cp.velocity.subLocal(v1);
				
				cp.normal.set(m_manifold.normal);
				cp.separation = mp0.separation;
				cp.id.set(mp0.id);
				listener.remove(cp);
			}
		}

	}
	
	// djm TODO move this to dynamics?
	private static Vec2 ECd = new Vec2();
	private static Vec2 ECc = new Vec2();
	private static Vec2 ECcLocal = new Vec2();
	//private static Vec2 ECcLocalSubV1 = new Vec2();
	public final static void CollideEdgeAndCircle(Manifold manifold,
			final EdgeShape edge, final XForm xf1,
			final CircleShape circle, final XForm xf2) {
		manifold.pointCount = 0;
		
		XForm.mulToOut(xf2, circle.getLocalPosition(), ECc);
		XForm.mulTransToOut(xf1, ECc, ECcLocal);
		
		Vec2 n = edge.getNormalVector();
		Vec2 v1 = edge.getVertex1();
		Vec2 v2 = edge.getVertex2();
		float radius = circle.getRadius();
		float separation;
		
		ECd.set(ECcLocal);
		ECd.subLocal(v1);
		
		float dirDist = Vec2.dot(ECd, edge.getDirectionVector());
		if (dirDist <= 0) {

			if (Vec2.dot(ECd, edge.getCorner1Vector()) < 0) {
				return;
			}
			XForm.mulToOut(xf1, v1, ECd);
			ECd.subLocal(ECc);
			ECd.negateLocal();
			// d = c.sub(XForm.mul(xf1, v1));
		} else if (dirDist >= edge.getLength()) {
			ECd.set(ECcLocal);
			ECd.subLocal(v2);
			if (Vec2.dot(ECd, edge.getCorner2Vector()) > 0) {
				return;
			}
			XForm.mulToOut(xf1, v2, ECd);
			ECd.subLocal(ECc);
			ECd.negateLocal();
			//d = c.sub(XForm.mul(xf1, v2));
		} else {
			separation = Vec2.dot(ECd, n);
			if (separation > radius || separation < -radius) {
				return;
			}
			separation -= radius;
			Mat22.mulToOut(xf1.R, n, manifold.normal);
			manifold.pointCount = 1;
			manifold.points[0].id.zero();// key = 0;
			manifold.points[0].separation = separation;
			// just use d as temp vec here, we don't need it any more
			ECd.set(manifold.normal);
			ECd.mulLocal(radius);
			ECc.subLocal(ECd);
			XForm.mulTransToOut(xf1, ECc, manifold.points[0].localPoint1);
			XForm.mulTransToOut(xf2, ECc, manifold.points[0].localPoint2);
			return;
		}
		
		float distSqr = Vec2.dot(ECd,ECd);
		if (distSqr > radius * radius) {
			return;
		}
		
		if (distSqr < Settings.EPSILON) {
			separation = -radius;
			Mat22.mulToOut(xf1.R, n, manifold.normal);
		} else {
			separation = ECd.normalize() - radius;
			manifold.normal.set(ECd);
		}
		
		manifold.pointCount = 1;
		manifold.points[0].id.zero();//key = 0;
		manifold.points[0].separation = separation;
		// just use d as temp vec here, we don't need it any more
		ECd.set(manifold.normal);
		ECd.mulLocal(radius);
		ECc.subLocal(ECd);
		//c.subLocal(manifold.normal.mul(radius));
		 XForm.mulTransToOut(xf1, ECc, manifold.points[0].localPoint1);
		 XForm.mulTransToOut(xf2, ECc, manifold.points[0].localPoint2);

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
