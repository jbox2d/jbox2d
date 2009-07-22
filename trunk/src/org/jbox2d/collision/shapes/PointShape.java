package org.jbox2d.collision.shapes;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.MassData;
import org.jbox2d.collision.Segment;
import org.jbox2d.collision.SegmentCollide;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.ObjectPool;
import org.jbox2d.common.RaycastResult;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

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

	/**
	 * @see Shape#computeAABB(AABB, XForm)
	 */
	@Override
	public void computeAABB(final AABB aabb, final XForm transform) {
		//Vec2 p = transform.position.add(Mat22.mul(transform.R, m_localPosition));
		final Vec2 p = ObjectPool.getVec2();
		Mat22.mulToOut(transform.R, m_localPosition, p);
		p.add(transform.position);
		aabb.lowerBound.set(p.x-Settings.EPSILON, p.y-Settings.EPSILON);
		aabb.upperBound.set(p.x+Settings.EPSILON, p.y+Settings.EPSILON);
		ObjectPool.returnVec2(p);
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

	/**
	 * @see Shape#computeSweptAABB(AABB, XForm, XForm)
	 */
	@Override
	public void computeSweptAABB(final AABB aabb, final XForm transform1, final XForm transform2) {
		final Vec2 sweptP1 = ObjectPool.getVec2();
		final Vec2 sweptP2 = ObjectPool.getVec2();
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
		
		ObjectPool.returnVec2(sweptP1);
		ObjectPool.returnVec2(sweptP2);
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
	
	/**
	 * @see Shape#testSegment(XForm, RaycastResult, Segment, float)
	 */
	@Override
	public SegmentCollide testSegment(final XForm xf, final RaycastResult out, final Segment segment, final float maxLambda){
		final Vec2 position = ObjectPool.getVec2();
		final Vec2 s = ObjectPool.getVec2();
		
		Mat22.mulToOut( xf.R, m_localPosition, position);
		position.addLocal(xf.position);
		s.set(segment.p1);
		s.subLocal(position);
		final float b = Vec2.dot(s, s);

		// Does the segment start inside the circle?
		if (b < 0.0f){
			ObjectPool.returnVec2(position);
			ObjectPool.returnVec2(s);
			return SegmentCollide.STARTS_INSIDE_COLLIDE;
		}

		final Vec2 r = ObjectPool.getVec2();

		// Solve quadratic equation.
		r.set(segment.p2).subLocal(segment.p1);
		final float c =  Vec2.dot(s, r);
		final float rr = Vec2.dot(r, r);
		final float sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0f || rr < Settings.EPSILON){
			ObjectPool.returnVec2(position);
			ObjectPool.returnVec2(s);
			ObjectPool.returnVec2(r);
			return SegmentCollide.MISS_COLLIDE;
		}

		// Find the point of intersection of the line with the circle.
		float a = -(c + (float)Math.sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0f <= a && a <= maxLambda * rr){
			a /= rr;
			out.lambda = a;
			out.normal.set(r).mulLocal(a).addLocal(s);
			out.normal.normalize();
			ObjectPool.returnVec2(position);
			ObjectPool.returnVec2(s);
			ObjectPool.returnVec2(r);
			return SegmentCollide.HIT_COLLIDE;
		}

		ObjectPool.returnVec2(position);
		ObjectPool.returnVec2(s);
		ObjectPool.returnVec2(r);
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
		m_sweepRadius = (float)Math.sqrt(dx*dx + dy*dy) - Settings.toiSlop;
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
