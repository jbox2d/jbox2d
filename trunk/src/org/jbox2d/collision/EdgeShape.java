package org.jbox2d.collision;

import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

public class EdgeShape extends Shape implements SupportsGenericDistance {
	//private updatesweepradius
	private Vec2 m_v1;
	private Vec2 m_v2;
	private Vec2 m_coreV1;
	private Vec2 m_coreV2;
	private float m_length;
	private Vec2 m_normal;
	private Vec2 m_direction;
	// Unit vector halfway between m_direction and m_prevEdge.m_direction:
	private Vec2 m_cornerDir1;
	// Unit vector halfway between m_direction and m_nextEdge.m_direction:
	private Vec2 m_cornerDir2;
	private boolean m_cornerConvex1;
	private boolean m_cornerConvex2;
	EdgeShape m_nextEdge;
	EdgeShape m_prevEdge;
	
	public EdgeShape(final Vec2 v1, final Vec2 v2, final ShapeDef def) {
		super(def);
		assert(def.type == ShapeType.EDGE_SHAPE);

		m_type = ShapeType.EDGE_SHAPE;
		
		m_prevEdge = null;
		m_nextEdge = null;
		
		m_v1 = v1;
		m_v2 = v2;
		
		m_direction = m_v2.sub(m_v1);
		m_length = m_direction.normalize();
		m_normal = new Vec2(m_direction.y, -m_direction.x);
		
		m_coreV1 = (m_normal.sub(m_direction)).mul(-Settings.toiSlop).add(m_v1);
		m_coreV2 = (m_normal.add(m_direction)).mul(-Settings.toiSlop).add(m_v2);
		
		m_cornerDir1 = m_normal.clone();
		m_cornerDir2 = m_normal.mul(-1.0f);
	}
	
	public void updateSweepRadius(final Vec2 center) {
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		float dx = m_coreV1.x - center.x;
		float dy = m_coreV1.y - center.y;
		float d1 = dx*dx+dy*dy;
		float dx2 = m_coreV2.x - center.x;
		float dy2 = m_coreV2.y - center.y;
		float d2 = dx2*dx2+dy2*dy2;
		m_sweepRadius = (float)Math.sqrt(d1 > d2 ? d1 : d2);
	}
	
	public boolean testPoint(final XForm transform, final Vec2 p) {
		return false;
	}
	
//	b2SegmentCollide b2EdgeShape::TestSegment(const b2XForm& transform,
//			float32* lambda,
//			b2Vec2* normal,
//			const b2Segment& segment,
//			float32 maxLambda) const
//{
//b2Vec2 r = segment.p2 - segment.p1;
//b2Vec2 v1 = b2Mul(transform, m_v1);
//b2Vec2 d = b2Mul(transform, m_v2); - v1;
//b2Vec2 n = b2Cross(d, 1.0f);
//
//const float32 k_slop = 100.0f * B2_FLT_EPSILON;
//float32 denom = -b2Dot(r, n);
//
//// Cull back facing collision and ignore parallel segments.
//if (denom > k_slop)
//{
//// Does the segment intersect the infinite line associated with this segment?
//b2Vec2 b = segment.p1 - v1;
//float32 a = b2Dot(b, n);
//
//if (0.0f <= a && a <= maxLambda * denom)
//{
//float32 mu2 = -r.x * b.y + r.y * b.x;
//
//// Does the segment intersect this segment?
//if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
//{
//a /= denom;
//n.Normalize();
//*lambda = a;
//*normal = n;
//return e_hitCollide;
//}
//}
//}
//
//return e_missCollide;
//}

	public void computeAABB(AABB aabb, final XForm transform) {
		Vec2 v1 = XForm.mul(transform, m_v1);
		Vec2 v2 = XForm.mul(transform, m_v2);
		aabb.lowerBound = Vec2.min(v1, v2);
		aabb.upperBound = Vec2.max(v1, v2);
	}
	
	public void computeSweptAABB(AABB aabb, final XForm transform1, final XForm transform2) {
		Vec2 v1 = XForm.mul(transform1, m_v1);
		Vec2 v2 = XForm.mul(transform1, m_v2);
		Vec2 v3 = XForm.mul(transform2, m_v1);
		Vec2 v4 = XForm.mul(transform2, m_v2);
		aabb.lowerBound = Vec2.min(Vec2.min(Vec2.min(v1, v2), v3), v4);
		aabb.upperBound = Vec2.max(Vec2.max(Vec2.max(v1, v2), v3), v4);
	}
	
	public void computeMass(MassData massData) {
		massData.mass = 0;
		massData.center = m_v1;

		// inertia about the local origin
		massData.I = 0;
	}
	
	public Vec2 support(final XForm xf, final Vec2 d) {
		Vec2 v1 = XForm.mul(xf, m_coreV1);
		Vec2 v2 = XForm.mul(xf, m_coreV2);
		return Vec2.dot(v1, d) > Vec2.dot(v2, d) ? v1 : v2;
	}
	
	public void setPrevEdge(EdgeShape edge, final Vec2 core, final Vec2 cornerDir, boolean convex) {
		m_prevEdge = edge;
		m_coreV1 = core;
		m_cornerDir1 = cornerDir;
		m_cornerConvex1 = convex;
	}
	
	public void setNextEdge(EdgeShape edge, final Vec2 core, final Vec2 cornerDir, boolean convex) {
		m_nextEdge = edge;
		m_coreV2 = core;
		m_cornerDir2 = cornerDir;
		m_cornerConvex2 = convex;
	}

	/** Linear distance from vertex1 to vertex2 */
	public float getLength() {
		return m_length;
	}

	/** Local position of vertex in parent body */
	public Vec2 getVertex1() {
		return m_v1;
	}

	/** Local position of vertex in parent body */
	public Vec2 getVertex2() {
		return m_v2;
	}

	/** "Core" vertex with TOI slop for b2Distance functions */
	public Vec2 getCoreVertex1() {
		return m_coreV1;
	}

	/** "Core" vertex with TOI slop for b2Distance functions */
	public Vec2 getCoreVertex2() {
		return m_coreV2;
	}

	/** Perpendecular unit vector point, pointing from the solid side to the empty side. */
	public Vec2 getNormalVector() {
		return m_normal;
	}

	/** Parallel unit vector, pointing from vertex1 to vertex2 */
	public Vec2 getDirectionVector() {
		return m_direction;
	}

	public Vec2 getCorner1Vector() {
		return m_cornerDir1;
	}

	public Vec2 getCorner2Vector() {
		return m_cornerDir2;
	}

	/** Get the next edge in the chain. */
	public EdgeShape getNextEdge() {
		return m_nextEdge;
	}

	/** Get the previous edge in the chain. */
	public EdgeShape getPrevEdge() {
		return m_prevEdge;
	}

	public Vec2 getFirstVertex(XForm xf) {
		return XForm.mul(xf, m_coreV1);
	}

	public boolean corner1IsConvex() {
		return m_cornerConvex1;
	}

	public boolean corner2IsConvex() {
		return m_cornerConvex2;
	}

}
