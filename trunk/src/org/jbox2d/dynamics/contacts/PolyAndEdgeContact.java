package org.jbox2d.dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.shapes.CollidePoly;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.collision.shapes.ShapeType;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.ContactListener;

public class PolyAndEdgeContact extends Contact implements ContactCreateFcn {
	Manifold m_manifold;
    ArrayList<Manifold> manifoldList = new ArrayList<Manifold>();

	public Contact create(Shape s1, Shape s2) {
		// TODO Auto-generated method stub
		return new PolyAndEdgeContact(s1,s2);
	}
	
	public PolyAndEdgeContact() {
		super();
		m_manifold = new Manifold();
        manifoldList.add(m_manifold);
		m_manifoldCount = 0;
	}
	 
	public PolyAndEdgeContact(Shape shape1, Shape shape2) {
		super(shape1, shape2);
		assert (m_shape1.getType() == ShapeType.POLYGON_SHAPE);
		assert (m_shape2.getType() == ShapeType.EDGE_SHAPE);
        m_manifold = new Manifold();
        m_manifoldCount = 0;
        manifoldList.add(m_manifold);
	}

	public static void Destroy(Contact contact) {
        ((PolyAndEdgeContact) contact).destructor();
    }
	
	public void destructor() {
		
	}
	
	@Override
	public Contact clone() {
		assert false: "Not yet implemented.";
    	return this;
	}

	// djm pooled
    private Manifold m0 = new Manifold();
    private Vec2 v1 = new Vec2();
	@Override
	public void evaluate(ContactListener listener) {
		Body b1 = m_shape1.getBody();
		Body b2 = m_shape2.getBody();

		m0.set( m_manifold);
		
		CollidePoly.collidePolyAndEdge(m_manifold, (PolygonShape)m_shape1, b1.getMemberXForm(), (EdgeShape)m_shape2, b2.getMemberXForm());

		boolean[] persisted = {false, false};

		ContactPoint cp = new ContactPoint();
		cp.shape1 = m_shape1;
		cp.shape2 = m_shape2;
		cp.friction = m_friction;
		cp.restitution = m_restitution;
		//TODO: add this once custom friction/restitution mixings are in place
		//cp.friction = b2MixFriction(m_shape1->GetFriction(), m_shape2->GetFriction());
		//cp.restitution = b2MixRestitution(m_shape1->GetRestitution(), m_shape2->GetRestitution());

		// Match contact ids to facilitate warm starting.
		if (m_manifold.pointCount > 0) {
			// Match old contact ids to new contact ids and copy the
			// stored impulses to warm start the solver.
			for (int i = 0; i < m_manifold.pointCount; ++i) {
				ManifoldPoint mp = m_manifold.points[i];
				mp.normalImpulse = 0.0f;
				mp.tangentImpulse = 0.0f;
				boolean found = false;
				ContactID id = mp.id;

				for (int j = 0; j < m0.pointCount; ++j) {
					if (persisted[j] == true) {
						continue;
					}

					ManifoldPoint mp0 = m0.points[j];

					if (mp0.id.isEqual(id)) {
						persisted[j] = true;
						mp.normalImpulse = mp0.normalImpulse;
						mp.tangentImpulse = mp0.tangentImpulse;

						// A persistent point.
						found = true;

						// Report persistent point.
						if (listener != null) {
							//cp.position = b1.getWorldLocation(mp.localPoint1);
							b1.getWorldLocationToOut(mp.localPoint1, cp.position);
    						//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
    	    				b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
    	    				//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
    	    				b2.getLinearVelocityFromLocalPointToOut(mp.localPoint2, cp.velocity);
    	    				//cp.velocity = v2.sub(v1);
    	    				cp.velocity.subLocal(v1);
    	    				
    						cp.normal.set(m_manifold.normal);
    						cp.separation = mp.separation;
    						cp.id.set(id);
							listener.persist(cp);
						}
						break;
					}
				}

				// Report added point.
				if (found == false && listener != null) {
					b1.getWorldLocationToOut(mp.localPoint1, cp.position);
					//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
    				b1.getLinearVelocityFromLocalPointToOut(mp.localPoint1, v1);
    				//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
    				b2.getLinearVelocityFromLocalPointToOut(mp.localPoint2, cp.velocity);
    				//cp.velocity = v2.sub(v1);
    				cp.velocity.subLocal(v1);
    				
					cp.normal.set(m_manifold.normal);
					cp.separation = mp.separation;
					cp.id.set(id);
					listener.add(cp);
				}
			}

			m_manifoldCount = 1;
		} else {
			m_manifoldCount = 0;
		}

		if (listener == null){
			return;
		}

		// Report removed points.
		for (int i = 0; i < m0.pointCount; ++i) {
			if (persisted[i]) {
				continue;
			}

			ManifoldPoint mp0 = m0.points[i];
    		b1.getWorldLocationToOut(mp0.localPoint1, cp.position);
			//Vec2 v1 = b1.getLinearVelocityFromLocalPoint(mp.localPoint1);
			b1.getLinearVelocityFromLocalPointToOut(mp0.localPoint1, v1);
			//Vec2 v2 = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
			b2.getLinearVelocityFromLocalPointToOut(mp0.localPoint2, cp.velocity);
			//cp.velocity = v2.sub(v1);
			cp.velocity.subLocal(v1);
			
			cp.normal.set(m_manifold.normal);
			cp.separation = mp0.separation;
			cp.id.set(mp0.id);
			listener.remove(cp);
		}
	}

	@Override
    public List<Manifold> getManifolds() {
        return manifoldList;
    }
}
