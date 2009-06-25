package org.jbox2d.collision.shapes;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

/**
 * An edge shape
 * @author daniel
 */
public class EdgeShape extends Shape {

	private final Vec2 m_v1 = new Vec2();
	private final Vec2 m_v2 = new Vec2();

	private float m_length;

	private final Vec2 m_normal = new Vec2();

	private final Vec2 m_direction = new Vec2();

	/**
	 * Unit vector halfway between m_direction and m_prevEdge.m_direction:
	 */
	private final Vec2 m_cornerDir1 = new Vec2();

	/**
	 * Unit vector halfway between m_direction and m_nextEdge.m_direction:
	 */
	private final Vec2 m_cornerDir2 = new Vec2();

	private boolean m_cornerConvex1;
	private boolean m_cornerConvex2;

	EdgeShape m_nextEdge;
	EdgeShape m_prevEdge;

	public EdgeShape() {
		m_type = ShapeType.EDGE_SHAPE;
		m_radius = Settings.polygonRadius;
		m_prevEdge = null;
		m_nextEdge = null;
	}

	@Override
	public void destructor(){
		if (m_prevEdge != null){
			m_prevEdge.m_nextEdge = null;
		}

		if (m_nextEdge != null){
			m_nextEdge.m_prevEdge = null;
		}
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

	/** Perpendicular unit vector point, pointing from the solid side to the empty side. */
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

	/**
	 * Initialize this edge using the two vertices.
	 * @param v1
	 * @param v2
	 */
	public void set( final Vec2 v1,  final Vec2 v2){
		m_v1.set(v1);
		m_v2.set(v2);

		m_direction.set(m_v2).subLocal(m_v1);
		m_length = m_direction.normalize();
		Vec2.crossToOut( m_direction, 1f, m_normal);

		m_cornerDir1.set(m_normal);
		m_cornerDir2.set(m_normal).negateLocal();
	}

	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 * @param transform
	 * @param p
	 * @return
	 */
	@Override
	public final boolean testPoint( final XForm transform,  final Vec2 p){
		// djm this could use some optimization.
		// TODO maybe try bitshifting?
		return false;
	}


	// djm pooled
	private final Vec2 r = new Vec2();
	private final Vec2 v1 = new Vec2();
	private final Vec2 d = new Vec2();
	private final Vec2 n = new Vec2();
	private final Vec2 b = new Vec2();
	/**
	 * @see Shape#testSegment(XForm, TestSegmentResult, Segment, float)
	 * @param transform
	 * @return
	 */
	@Override
	public final SegmentCollide testSegment(final XForm transform, final TestSegmentResult out, final Segment segment, final float maxLambda){
		r.set(segment.p2).subLocal(segment.p1);
		XForm.mulToOut( transform, m_v1, v1);
		XForm.mulToOut( transform, m_v2, d);
		Vec2.crossToOut( d, 1f, n);

		final float k_slop = 100.0f * Settings.EPSILON;
		final float denom = -Vec2.dot(r, n);

		// Cull back facing collision and ignore parallel segments.
		if (denom > k_slop)
		{
			// Does the segment intersect the infinite line associated with this segment?
			b.set(segment.p1).subLocal(v1);
			float a = Vec2.dot(b, n);

			if (0.0f <= a && a <= maxLambda * denom)
			{
				final float mu2 = -r.x * b.y + r.y * b.x;

				// Does the segment intersect this segment?
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0f + k_slop))
				{
					a /= denom;
					n.normalize();
					out.lambda = a;
					out.normal.set(n);
					return SegmentCollide.HIT_COLLIDE;
				}
			}
		}

		return SegmentCollide.MISS_COLLIDE;
	}


	// djm pooled, and from above
	private static final Vec2 v2 = new Vec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public final void computeAABB(final AABB aabb, final XForm transform) {
		/*Vec2 v1 = Mul(transform, m_v1);
		Vec2 v2 = Mul(transform, m_v2);

		Vec2 r(m_radius, m_radius);
		aabb->lowerBound = Min(v1, v2) - r;
		aabb->upperBound = Max(v1, v2) + r;*/

		XForm.mulToOut( transform, m_v1, v1);
		XForm.mulToOut( transform, m_v2, v2);
		r.set(m_radius, m_radius);

		Vec2.minToOut( v1, v2, aabb.lowerBound);
		Vec2.maxToOut( v1, v2, aabb.upperBound);
		aabb.lowerBound.subLocal(r);
		aabb.upperBound.addLocal(r);
	}

	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	@Override
	public final void computeMass(final MassData massData, final float density){
		massData.mass = 0.0f;
		massData.center.set(m_v1);
		massData.I = 0.0f;
	}


	// djm pooled, some from above
	private static final Vec2 v0 = new Vec2();
	private static final Vec2 temp = new Vec2();
	private static final Vec2 e1 = new Vec2();
	private static final Vec2 e2 = new Vec2();
	/**
	 * WARNING: This only gives a consistent and sensible answer when when summed over a body only contains loops of edges
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 * @param normal
	 * @param offset
	 * @param xf
	 * @param c
	 * @return
	 */
	@Override
	public final float computeSubmergedArea(	 final Vec2 normal,
	                                        	 final float offset,
	                                        	 final XForm xf,
	                                        	 final Vec2 c){
		//Note that v0 is independent of any details of the specific edge
		//We are relying on v0 being consistent between multiple edges of the same body
		v0.set(normal).mulLocal( offset);
		//Vec2 v0 = xf.position + (offset - Dot(normal, xf.position)) * normal;

		XForm.mulToOut(xf, m_v1, v1);
		XForm.mulToOut(xf, m_v2, v2);

		final float d1 = Vec2.dot(normal, v1) - offset;
		final float d2 = Vec2.dot(normal, v2) - offset;

		if (d1 > 0.0f){
			if (d2 > 0.0f){
				return 0.0f;
			}
			else{
				temp.set(v2).mulLocal(d1 / (d1 - d2));
				v1.mulLocal( -d2 / (d1 - d2));
				v1.addLocal(temp);
				//v1 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
			}
		}
		else{
			if (d2 > 0.0f){
				temp.set(v1).mulLocal(-d2 / (d1 - d2));
				v2.mulLocal(d1 / (d1 - d2));
				v2.addLocal( temp);
				//v2 = -d2 / (d1 - d2) * v1 + d1 / (d1 - d2) * v2;
			}
			else{
				//Nothing
			}
		}

		// v0,v1,v2 represents a fully submerged triangle
		final float k_inv3 = 1.0f / 3.0f;

		// Area weighted centroid
		c.set(v0).addLocal( v1).addLocal( v2);
		c.mulLocal( k_inv3);
		//*c = k_inv3 * (v0 + v1 + v2);

		e1.set(v1).subLocal(v0);
		e2.set(v2).subLocal(v0);

		return 0.5f * Vec2.cross(e1, e2);
	}

	/**
	 * @see Shape#computeSweepRadius(Vec2)
	 * @param pivot
	 * @return
	 */
	@Override
	public final float computeSweepRadius( final Vec2 pivot){
		final float ds1 = MathUtils.distanceSquared(m_v1, pivot);
		final float ds2 = MathUtils.distanceSquared(m_v2, pivot);
		return (float) Math.sqrt(MathUtils.max(ds1, ds2));
	}


	/**
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 */
	public final int getSupport( final Vec2 d){
		return Vec2.dot(m_v1, d) > Vec2.dot(m_v2, d) ? 0 : 1;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 */
	public final Vec2 getSupportVertex( final Vec2 d){
		return Vec2.dot(m_v1, d) > Vec2.dot(m_v2, d) ? m_v1 : m_v2;
	}

	/**
	 * Get the vertex count.
	 */
	public final int getVertexCount()  {
		return 2;
	}

	/**
	 * Get a vertex by index. Used by Distance.
	 * @param index
	 * @return
	 */
	public final Vec2 getVertex(final int index){
		assert(0 <= index && index < 2);
		return index==0 ? m_v1 : m_v2;
	}

	/**
	 * Get the next edge in the chain.
	 * @return
	 */
	public final EdgeShape getNextEdge(){
		return m_nextEdge;
	}

	/**
	 * Get the previous edge in the chain.
	 */
	public final EdgeShape getPrevEdge(){
		return m_prevEdge;
	}

	public final void setPrevEdge(final EdgeShape edge,  final Vec2 cornerDir, final boolean convex){
		m_prevEdge = edge;
		m_cornerDir1.set(cornerDir);
		m_cornerConvex1 = convex;
	}

	public final void setNextEdge(final EdgeShape edge,  final Vec2 cornerDir, final boolean convex){
		m_nextEdge = edge;
		m_cornerDir2.set(cornerDir);
		m_cornerConvex2 = convex;
	}

	public boolean corner1IsConvex() {
		return m_cornerConvex1;
	}

	public boolean corner2IsConvex() {
		return m_cornerConvex2;
	}
}
