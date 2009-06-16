package org.jbox2d.dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.EdgeShape;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.PolygonShape;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
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
		
		CollidePolyAndEdge(m_manifold, (PolygonShape)m_shape1, b1.getMemberXForm(), (EdgeShape)m_shape2, b2.getMemberXForm());

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
							cp.position = b1.getWorldLocation(mp.localPoint1);
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
    				// djm cp.velocity isn't instantiated in the constructor,
    				// so we just create it here
    				cp.velocity = b2.getLinearVelocityFromLocalPoint(mp.localPoint2);
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
	
	// djm TODO shouldn't this be in a static collision class??
	// djm pooled
	private static Vec2 PEv1 = new Vec2();
	private static Vec2 PEv2 = new Vec2();
	private static Vec2 PEn = new Vec2();
	private static Vec2 PEv1Local = new Vec2();
	private static Vec2 PEv2Local = new Vec2();
	private static Vec2 PEnLocal = new Vec2();
	private static Vec2 temp = new Vec2();
	private static Vec2 temp2 = new Vec2();
	public static final void CollidePolyAndEdge(Manifold manifold,
			  final PolygonShape polygon, 
			  final XForm xf1,
			  final EdgeShape edge, 
			  final XForm xf2) {
		manifold.pointCount = 0;
		XForm.mulToOut(xf2, edge.getVertex1(), PEv1);
		XForm.mulToOut(xf2, edge.getVertex2(), PEv2);
		Mat22.mulToOut(xf2.R, edge.getNormalVector(), PEn);
		XForm.mulTransToOut(xf1, PEv1, PEv1Local);
		XForm.mulTransToOut(xf1, PEv2, PEv2Local);
		Mat22.mulTransToOut(xf1.R, PEn, PEnLocal);

		float separation1;
		int separationIndex1 = -1; // which normal on the poly found the shallowest depth?
		float separationMax1 = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		float separation2;
		int separationIndex2 = -1; // which normal on the poly found the shallowest depth?
		float separationMax2 = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		float separationMax = -Float.MAX_VALUE; // the shallowest depth of edge in poly
		boolean separationV1 = false; // is the shallowest depth from edge's v1 or v2 vertex?
		int separationIndex = -1; // which normal on the poly found the shallowest depth?
		
		int vertexCount = polygon.getVertexCount();
		final Vec2[] vertices = polygon.getVertices();
		final Vec2[] normals = polygon.getNormals();
		
		int enterStartIndex = -1; // the last poly vertex above the edge
		int enterEndIndex = -1; // the first poly vertex below the edge
		int exitStartIndex = -1; // the last poly vertex below the edge
		int exitEndIndex = -1; // the first poly vertex above the edge
		//int deepestIndex;
		
		// the "N" in the following variables refers to the edge's normal. 
		// these are projections of poly vertices along the edge's normal, 
		// a.k.a. they are the separation of the poly from the edge. 
		float prevSepN = 0.0f;
		float nextSepN = 0.0f;
		float enterSepN = 0.0f; // the depth of enterEndIndex under the edge (stored as a separation, so it's negative)
		float exitSepN = 0.0f; // the depth of exitStartIndex under the edge (stored as a separation, so it's negative)
		float deepestSepN = Float.MAX_VALUE; // the depth of the deepest poly vertex under the end (stored as a separation, so it's negative)
		
		
		// for each poly normal, get the edge's depth into the poly. 
		// for each poly vertex, get the vertex's depth into the edge. 
		// use these calculations to define the remaining variables declared above.
		temp.set( vertices[vertexCount-1]);
		temp.subLocal( PEv1Local);
		prevSepN = Vec2.dot(temp, PEnLocal);
		
		for (int i = 0; i < vertexCount; i++) {
			temp.set(PEv1Local);
			temp.subLocal( vertices[i]);
			separation1 = Vec2.dot(temp, normals[i]);
			temp.set(PEv2Local);
			temp.subLocal( vertices[i]);
			separation2 = Vec2.dot(temp, normals[i]);
			if (separation2 < separation1) {
				if (separation2 > separationMax) {
					separationMax = separation2;
					separationV1 = false;
					separationIndex = i;
				}
			} else {
				if (separation1 > separationMax) {
					separationMax = separation1;
					separationV1 = true;
					separationIndex = i;
				}
			}
			if (separation1 > separationMax1) {
				separationMax1 = separation1;
				separationIndex1 = i;
			}
			if (separation2 > separationMax2) {
				separationMax2 = separation2;
				separationIndex2 = i;
			}
		
			temp.set( vertices[i]);
			temp.subLocal( PEv1Local);
			
			nextSepN = Vec2.dot(temp, PEnLocal);
			if (nextSepN >= 0.0f && prevSepN < 0.0f) {
				exitStartIndex = (i == 0) ? vertexCount-1 : i-1;
				exitEndIndex = i;
				exitSepN = prevSepN;
			} else if (nextSepN < 0.0f && prevSepN >= 0.0f) {
				enterStartIndex = (i == 0) ? vertexCount-1 : i-1;
				enterEndIndex = i;
				enterSepN = nextSepN;
			}
			if (nextSepN < deepestSepN) {
				deepestSepN = nextSepN;
				//deepestIndex = i;
			}
			prevSepN = nextSepN;
		}
		
		if (enterStartIndex == -1) {
			// poly is entirely below or entirely above edge, return with no contact:
			return;
		}
		if (separationMax > 0.0f) {
			// poly is laterally disjoint with edge, return with no contact:
			return;
		}
		
		// if the poly is near a convex corner on the edge
		if ((separationV1 && edge.corner1IsConvex()) || (!separationV1 && edge.corner2IsConvex())) {
			// if shallowest depth was from edge into poly, 
			// use the edge's vertex as the contact point:
			if (separationMax > deepestSepN + Settings.linearSlop) {
				// if -normal angle is closer to adjacent edge than this edge, 
				// let the adjacent edge handle it and return with no contact:
				if (separationV1) {
					Mat22.mulToOut( xf2.R, edge.getCorner1Vector(), temp);
					Mat22.mulTransToOut(xf1.R, temp, temp);
					if (Vec2.dot(normals[separationIndex1], temp) >= 0.0f) {
						return;
					}
				} else {
					Mat22.mulToOut( xf2.R, edge.getCorner2Vector(), temp);
					Mat22.mulTransToOut(xf1.R, temp, temp);
					if (Vec2.dot(normals[separationIndex2], temp) <= 0.0f) {
						return;
					}
				}
			
				manifold.pointCount = 1;
				Mat22.mulToOut(xf1.R, normals[separationIndex], manifold.normal);
				manifold.points[0].separation = separationMax;
				manifold.points[0].id.features.incidentEdge = separationIndex;
				manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
				manifold.points[0].id.features.referenceEdge = 0;
				manifold.points[0].id.features.flip = 0;
				if (separationV1) {
					manifold.points[0].localPoint1.set( PEv1Local);
					manifold.points[0].localPoint2.set( edge.getVertex1());
				} else {
					manifold.points[0].localPoint1.set(PEv2Local);
					manifold.points[0].localPoint2.set(edge.getVertex2());
				}
				return;
			}
		}
		
		// We're going to use the edge's normal now.
		temp.set( PEn);
		temp.mulLocal( -1f);
		
		manifold.normal.set( temp);// = n.mul(-1.0f);
		
		// Check whether we only need one contact point.
		if (enterEndIndex == exitStartIndex) {
			manifold.pointCount = 1;
			manifold.points[0].id.features.incidentEdge = enterEndIndex;
			manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
			manifold.points[0].id.features.referenceEdge = 0;
			manifold.points[0].id.features.flip = 0;
			manifold.points[0].localPoint1.set(vertices[enterEndIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[enterEndIndex]), manifold.points[0].localPoint2);
			manifold.points[0].separation = enterSepN;
			return;
		}
		
		manifold.pointCount = 2;
		
		// dirLocal should be the edge's direction vector, but in the frame of the polygon.
		Vec2.crossToOut(PEnLocal, -1.0f, temp); // TODO: figure out why this optimization didn't work
		//Vec2 dirLocal = XForm.mulT(xf1.R, XForm.mul(xf2.R, edge.GetDirectionVector()));
		temp2.set( vertices[enterEndIndex]);
		temp2.subLocal( PEv1Local);
		
		float dirProj1 = Vec2.dot(temp, temp2);
		float dirProj2 = 0.0f;
		
		// The contact resolution is more robust if the two manifold points are 
		// adjacent to each other on the polygon. So pick the first two poly
		// vertices that are under the edge:
		temp2.set( vertices[exitStartIndex]);
		temp2.subLocal( PEv1Local);
		
		exitEndIndex = (enterEndIndex == vertexCount - 1) ? 0 : enterEndIndex + 1;
		if (exitEndIndex != exitStartIndex) {
			exitStartIndex = exitEndIndex;
			exitSepN = Vec2.dot(PEnLocal, temp2);
		}
		// temp is dirLocal still
		dirProj2 = Vec2.dot(temp, temp2);
		
		manifold.points[0].id.features.incidentEdge = enterEndIndex;
		manifold.points[0].id.features.incidentVertex = Collision.NULL_FEATURE;
		manifold.points[0].id.features.referenceEdge = 0;
		manifold.points[0].id.features.flip = 0;
		
		if (dirProj1 > edge.getLength()) {
			manifold.points[0].localPoint1.set(PEv2Local);
			manifold.points[0].localPoint2.set(edge.getVertex2());
			float ratio = (edge.getLength() - dirProj2) / (dirProj1 - dirProj2);
			if (ratio > 100.0f * Settings.EPSILON && ratio < 1.0f) {
				manifold.points[0].separation = exitSepN * (1.0f - ratio) + enterSepN * ratio;
			} else {
				manifold.points[0].separation = enterSepN;
			}
		} else {
			manifold.points[0].localPoint1.set(vertices[enterEndIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[enterEndIndex]), manifold.points[0].localPoint2);
			manifold.points[0].separation = enterSepN;
		}
		
		manifold.points[1].id.features.incidentEdge = exitStartIndex;
		manifold.points[1].id.features.incidentVertex = Collision.NULL_FEATURE;
		manifold.points[1].id.features.referenceEdge = 0;
		manifold.points[1].id.features.flip = 0;
		
		if (dirProj2 < 0.0f) {
			manifold.points[1].localPoint1.set(PEv1Local);
			manifold.points[1].localPoint2.set(edge.getVertex1());
			float ratio = (-dirProj1) / (dirProj2 - dirProj1);
			if (ratio > 100.0f * Settings.EPSILON && ratio < 1.0f) {
				manifold.points[1].separation = enterSepN * (1.0f - ratio) + exitSepN * ratio;
			} else {
				manifold.points[1].separation = exitSepN;
			}
		} else {
			manifold.points[1].localPoint1.set(vertices[exitStartIndex]);
			XForm.mulTransToOut(xf2, XForm.mul(xf1, vertices[exitStartIndex]), manifold.points[1].localPoint2);
			manifold.points[1].separation = exitSepN;
		}
	}

	
	

	@Override
    public List<Manifold> getManifolds() {
        return manifoldList;
    }
}
