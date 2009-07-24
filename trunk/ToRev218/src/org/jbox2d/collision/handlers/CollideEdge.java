package org.jbox2d.collision.handlers;

import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.Manifold.ManifoldType;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

public class CollideEdge {

	//private final static Vec2 ECd = new Vec2();
	private final static Vec2 ECc = new Vec2();
	private final static Vec2 ECcLocal = new Vec2();
	private final static Vec2 temp1 = new Vec2();
	private final static Vec2 temp2 = new Vec2();

	/**
	 * This implements 2-sided edge vs circle collision.
	 * @param manifold
	 * @param edge
	 * @param xf1
	 * @param circle
	 * @param xf2
	 */
	public final static void collideEdgeAndCircle(final Manifold manifold,
	                                              final EdgeShape edge, final XForm xf1,
	                                              final CircleShape circle, final XForm xf2) {
		manifold.m_pointCount = 0;

		XForm.mulToOut(xf2, circle.m_p, ECc);
		XForm.mulTransToOut(xf1, ECc, ECcLocal);

		final Vec2 n = edge.getNormalVector();
		final Vec2 v1 = edge.getVertex1();
		final Vec2 v2 = edge.getVertex2();
		final float radius = circle.m_radius + edge.m_radius;

		// Barycentric coordinates
		temp1.set(ECcLocal).subLocal(v1);
		temp2.set(v2).subLocal(v1);
		float u1 = Vec2.dot(temp1, temp2);
		temp1.set(ECcLocal).subLocal(v2);
		temp2.set(v1).subLocal(v2);
		float u2 = Vec2.dot(temp1, temp2);

		if (u1 <= 0.0f){
			// Behind v1
			if (MathUtils.distanceSquared(ECcLocal, v1) > radius * radius){
				return;
			}

			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(ECcLocal).subLocal(v1);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v1);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else if (u2 <= 0.0f){
			// Ahead of v2
			if (MathUtils.distanceSquared(ECcLocal, v2) > radius * radius)
			{
				return;
			}

			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			manifold.m_localPlaneNormal.set(ECcLocal).subLocal(v2);
			manifold.m_localPlaneNormal.normalize();
			manifold.m_localPoint.set(v2);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}
		else{
			temp1.set(ECcLocal).subLocal(v1);
			float separation = Vec2.dot(temp1, n);
			if (separation < -radius || radius < separation)
			{
				return;
			}
			
			manifold.m_pointCount = 1;
			manifold.m_type = ManifoldType.e_faceA;
			if(separation < 0f){
				manifold.m_localPlaneNormal.set(n).negateLocal();
			}else{
				manifold.m_localPlaneNormal.set(n);
			}
			manifold.m_localPoint.set(v1).addLocal(v2).mulLocal(0.5f);
			manifold.m_points[0].m_localPoint.set(circle.m_p);
			manifold.m_points[0].m_id.key = 0;
		}

		/*final float distSqr = Vec2.dot(ECd,ECd);
		if (distSqr > radius * radius) {
			return;
		}

		if (distSqr < Settings.EPSILON) {
			separation = -radius;
			Mat22.mulToOut(xf1.R, n, manifold.m_normal);
		} else {
			separation = ECd.normalize() - radius;
			manifold.m_normal.set(ECd);
		}

		manifold.m_pointCount = 1;
		manifold.m_points[0].id.zero();//key = 0;
		manifold.m_points[0].separation = separation;
		// just use d as temp vec here, we don't need it any more
		ECd.set(manifold.m_normal);
		ECd.mulLocal(radius);
		ECc.subLocal(ECd);
		//c.subLocal(manifold.m_normal.mul(radius));
		XForm.mulTransToOut(xf1, ECc, manifold.m_points[0].localPoint1);
		XForm.mulTransToOut(xf2, ECc, manifold.m_points[0].localPoint2);
		*/
	}
	
	/**
	 * Polygon versus 2-sided edge
	 */
	private static final PolygonShape polygonB = new PolygonShape();
	public static final void collidePolyAndEdge(final Manifold manifold,
            									final PolygonShape polygon,
            									final XForm xf1,
            									final EdgeShape edge,
            									final XForm xf2) {
		polygonB.setAsEdge(edge.getVertex1(), edge.getVertex2());
		CollidePoly.collidePolygons(manifold, polygon, xf1, polygonB, xf2);
	}
	
	
	// djm pooled
	/*private final static Vec2 PEv1 = new Vec2();
	private final static Vec2 PEv2 = new Vec2();
	private final static Vec2 PEn = new Vec2();
	private final static Vec2 PEv1Local = new Vec2();
	private final static Vec2 PEv2Local = new Vec2();
	private final static Vec2 PEnLocal = new Vec2();
	private final static Vec2 temp = new Vec2();*/
	/**
	 * puts collision information into the manifold about a collision between
	 * a polygon and an edge
	 * @param manifold
	 * @param polygon
	 * @param xf1
	 * @param edge
	 * @param xf2
	 */
	/*public static final void collidePolyAndEdge(final Manifold manifold,
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

		final int vertexCount = polygon.getVertexCount();
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

		final float dirProj1 = Vec2.dot(temp, temp2);
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
			final float ratio = (edge.getLength() - dirProj2) / (dirProj1 - dirProj2);
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
			final float ratio = (-dirProj1) / (dirProj2 - dirProj1);
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
	}*/
}
