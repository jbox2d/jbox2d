package org.jbox2d.collision.shapes;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;
import org.jbox2d.pooling.TLVec2;

/**
 * Point shape.  Like a circle shape of zero radius, except
 * that it has a finite mass.
 *
 */
public class PointShape extends Shape {
	public final Vec2 m_localPosition;
	public float m_mass;

	public PointShape(final ShapeDef def) {
		super(def);
		assert(def.type == ShapeType.POINT_SHAPE);
		final PointDef pointDef = (PointDef)def;
		m_type = ShapeType.POINT_SHAPE;
		m_localPosition = pointDef.localPosition.clone();
		m_mass = pointDef.mass;
	}
	
	// djm pooling
	private static final TLVec2 tlP = new TLVec2();
	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public void computeAABB(final AABB aabb, final XForm transform) {
		//Vec2 p = transform.position.add(Mat22.mul(transform.R, m_localPosition));
		final Vec2 p = tlP.get();
		Mat22.mulToOut(transform.R, m_localPosition, p);
		p.add(transform.position);
		aabb.lowerBound.set(p.x-Settings.EPSILON, p.y-Settings.EPSILON);
		aabb.upperBound.set(p.x+Settings.EPSILON, p.y+Settings.EPSILON);
	}

	/**
	 * @see Shape#computeMass(MassData)
	 */
	@Override
	public void computeMass(final MassData massData) {
		massData.mass = m_mass;
		massData.center.set(m_localPosition);
		massData.I = 0.0f;
	}

	// djm pooling
	private static final TLVec2 tlSwept1 = new TLVec2();
	private static final TLVec2 tlSwept2 = new TLVec2();
	/**
	 * @see Shape#computeSweptAABB(AABB, XForm, XForm)
	 */
	@Override
	public void computeSweptAABB(final AABB aabb, final XForm transform1, final XForm transform2) {
		final Vec2 sweptP1 = tlSwept1.get();
		final Vec2 sweptP2 = tlSwept2.get();
		//Vec2 p1 = transform1.position.add(Mat22.mul(transform1.R, m_localPosition));
		//Vec2 p2 = transform2.position.add(Mat22.mul(transform2.R, m_localPosition));
		Mat22.mulToOut( transform2.R, m_localPosition, sweptP1);
		Mat22.mulToOut( transform2.R, m_localPosition, sweptP2);

		Vec2.minToOut( sweptP1, sweptP2, aabb.lowerBound);
		Vec2.maxToOut( sweptP1, sweptP2, aabb.upperBound);

		aabb.lowerBound.x -= Settings.EPSILON;
		aabb.lowerBound.y -= Settings.EPSILON;

		aabb.upperBound.x += Settings.EPSILON;
		aabb.upperBound.y += Settings.EPSILON;
	}

	/**
	 * @see Shape#testPoint(XForm, Vec2)
	 */
	@Override
	public boolean testPoint(final XForm xf, final Vec2 p) {
		// TODO djm: could use more optimization.
		// we could probably use bit shifting
		return false;
	}

	// Collision Detection in Interactive 3D Environments by Gino van den Bergen
	// From Section 3.1.2
	// x = s + a * r
	// norm(x) = radius
	// djm pooled
	private static final TLVec2 tlS = new TLVec2();
	private static final TLVec2 tlPosition = new TLVec2();
	private static final TLVec2 tlR = new TLVec2();
	/**
	 * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
	 */
	@Override
	public SegmentCollide testSegment(final XForm xf, final RaycastResult out, final Segment segment, final float maxLambda){
		final Vec2 position = tlPosition.get();
		final Vec2 s = tlS.get();
		
		Mat22.mulToOut( xf.R, m_localPosition, position);
		position.addLocal(xf.position);
		s.set(segment.p1);
		s.subLocal(position);
		final float b = Vec2.dot(s, s);

		// Does the segment start inside the circle?
		if (b < 0.0f){
			return SegmentCollide.STARTS_INSIDE_COLLIDE;
		}

		final Vec2 r = tlR.get();

		// Solve quadratic equation.
		r.set(segment.p2).subLocal(segment.p1);
		final float c =  Vec2.dot(s, r);
		final float rr = Vec2.dot(r, r);
		final float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < Settings.EPSILON){
			return SegmentCollide.MISS_COLLIDE;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + MathUtils.sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= maxLambda * rr){
			a /= rr;
			out.lambda = a;
			out.normal.set(r).mulLocal(a).addLocal(s);
			out.normal.normalize();
			return SegmentCollide.HIT_COLLIDE;
		}

		return SegmentCollide.MISS_COLLIDE;
	}

	/**
	 * @see Shape#updateSweepRadius(Vec2)
	 */
	// djm optimized
	@Override
	public void updateSweepRadius(final Vec2 center) {
		//Vec2 d = m_localPosition.sub(center);
		final float dx = m_localPosition.x - center.x;
		final float dy = m_localPosition.y - center.y;
		m_sweepRadius = MathUtils.sqrt(dx*dx + dy*dy) - Settings.toiSlop;
	}

	/**
	 * @return a copy of local position
	 */
	public Vec2 getLocalPosition() {
		return m_localPosition.clone();
	}

	/**
	 * This is the member variable for the local position.
	 * Don't change this.
	 * @return
	 */
	public Vec2 getMemberLocalPosition(){
		return m_localPosition;
	}

	public float getMass() {
		return m_mass;
	}

}
