package org.jbox2d.collision.shapes;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.structs.SegmentCollide;
import org.jbox2d.collision.structs.ShapeType;
import org.jbox2d.collision.structs.TestSegmentResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

/**
 * An edge shape
 * @author daniel
 */
public class EdgeShape extends Shape {

	protected final Vec2 m_v1 = new Vec2();
	protected final Vec2 m_v2 = new Vec2();

	protected final Vec2 m_coreV1 = new Vec2();
	protected final Vec2 m_coreV2 = new Vec2();
	
	protected float m_length;

	protected final Vec2 m_normal = new Vec2();

	private final Vec2 m_direction = new Vec2();

	/**
	 * Unit vector halfway between m_direction and m_prevEdge.m_direction:
	 */
	protected final Vec2 m_cornerDir1 = new Vec2();

	/**
	 * Unit vector halfway between m_direction and m_nextEdge.m_direction:
	 */
	protected final Vec2 m_cornerDir2 = new Vec2();

	protected boolean m_cornerConvex1;
	protected boolean m_cornerConvex2;

	protected EdgeShape m_nextEdge;
	protected EdgeShape m_prevEdge;

	public EdgeShape(final Vec2 v1, final Vec2 v2, ShapeDef def) {
		super(def);
		assert(def.type == ShapeType.EDGE_SHAPE);
		m_type = ShapeType.EDGE_SHAPE;
		
		m_prevEdge = null;
		m_nextEdge = null;
		
		m_v1.set(v1);
		m_v2.set(v2);
		
		m_direction.set(m_v2).subLocal(m_v1);
		m_length = m_direction.normalize();
		m_normal.set(m_direction.y, -m_direction.x);
		
		// = -b2_toiSlop * (m_normal - m_direction) + m_v1;
		m_coreV1.set( m_normal).subLocal(m_direction).mulLocal( -Settings.toiSlop).addLocal(m_v1);
		// = -b2_toiSlop * (m_normal + m_direction) + m_v2;
		m_coreV2.set( m_normal).subLocal(m_direction).mulLocal( -Settings.toiSlop).addLocal(m_v2);
		
		m_cornerDir1.set(m_normal);
		m_cornerDir2.set(m_normal).negateLocal();
	}

	/**
	 * Initialize this edge using the two vertices.
	 * @param v1
	 * @param v2
	 */
	public void set( final Vec2 v1,  final Vec2 v2){
		m_prevEdge = null;
		m_nextEdge = null;
		
		m_v1.set(v1);
		m_v2.set(v2);
		
		m_direction.set(m_v2).subLocal(m_v1);
		m_length = m_direction.normalize();
		m_normal.set(m_direction.y, -m_direction.x);
		
		// = -b2_toiSlop * (m_normal - m_direction) + m_v1;
		m_coreV1.set( m_normal).subLocal(m_direction).mulLocal( -Settings.toiSlop).addLocal(m_v1);
		// = -b2_toiSlop * (m_normal + m_direction) + m_v2;
		m_coreV2.set( m_normal).subLocal(m_direction).mulLocal( -Settings.toiSlop).addLocal(m_v2);
		
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

		Vec2.minToOut( v1, v2, aabb.lowerBound);
		Vec2.maxToOut( v1, v2, aabb.upperBound);
	}
	
	// djm pooled, and from above
	private static final Vec2 v3 = new Vec2();
	private static final Vec2 v4 = new Vec2();
	
	public void computeSweptAABB( AABB aabb, XForm transform1, XForm transform2){
		XForm.mulToOut( transform1, m_v1, v1);
		XForm.mulToOut( transform1, m_v2, v2);
		XForm.mulToOut( transform2, m_v1, v3);
		XForm.mulToOut( transform2, m_v2, v4);
		
		Vec2.minToOut( v1, v2, aabb.lowerBound);
		Vec2.minToOut( aabb.lowerBound, v3, aabb.lowerBound);
		Vec2.minToOut( aabb.lowerBound, v4, aabb.lowerBound);
		
		Vec2.maxToOut( v1, v2, aabb.upperBound);
		Vec2.maxToOut( aabb.upperBound, v3, aabb.upperBound);
		Vec2.maxToOut( aabb.upperBound, v4, aabb.upperBound);
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

	/** "Core" vertex with TOI slop for b2Distance functions: */
	public Vec2 getCoreVertex1() {
		return m_coreV1;
	}
	
	/** "Core" vertex with TOI slop for b2Distance functions: */
	public Vec2 getCoreVertex2() {
		return m_coreV2;
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

	public boolean corner1IsConvex() {
		return m_cornerConvex1;
	}

	public boolean corner2IsConvex() {
		return m_cornerConvex2;
	}
	
	public Vec2 getFirstVertex( XForm xf){
		return XForm.mul(xf, m_coreV1);
	}
	
	public void getFirstVertexToOut( XForm xf, Vec2 out){
		XForm.mulToOut( xf, m_coreV1, out);
	}
	
	//djm pooled, from above
	
	public Vec2 support( XForm xf, Vec2 d){
		XForm.mulToOut( xf, m_coreV1, v1);
		XForm.mulToOut( xf, m_coreV2, v2);
		return Vec2.dot(v1, d) > Vec2.dot( v2, d) ? v1 : v2;
	}
	
	public void supportToOut( XForm xf, Vec2 d, Vec2 out){
		XForm.mulToOut( xf, m_coreV1, v1);
		XForm.mulToOut( xf, m_coreV2, v2);
		if(Vec2.dot(v1, d) > Vec2.dot( v2, d) ){
			out.set(v1);
		}else{
			out.set(v2);			
		}
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

	public final void setPrevEdge(final EdgeShape edge,  final Vec2 core, final Vec2 cornerDir, final boolean convex){
		m_prevEdge = edge;
		m_coreV1.set(core);
		m_cornerDir1.set(cornerDir);
		m_cornerConvex1 = convex;
	}

	public final void setNextEdge(final EdgeShape edge,  final Vec2 core, final Vec2 cornerDir, final boolean convex){
		m_nextEdge = edge;
		m_coreV2.set(core);
		m_cornerDir2.set(cornerDir);
		m_cornerConvex2 = convex;
	}
	
	// djm pooled, from above
	
	protected void updateSweepRadius(Vec2 center){
		// Update the sweep radius (maximum radius) as measured from
		// a local center point.
		d.set(m_coreV1).subLocal(center);
		float d1 = Vec2.dot(d,d);
		d.set(m_coreV2).subLocal(center);
		float d2 = Vec2.dot(d,d);
		m_sweepRadius = (float) Math.sqrt(d1 > d2 ? d1 : d2);
	}
}
