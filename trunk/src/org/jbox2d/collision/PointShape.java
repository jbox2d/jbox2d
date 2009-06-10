package org.jbox2d.collision;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.XForm;

/**
 * Point shape.  Like a circle shape of zero radius, except
 * that it has a finite mass.
 *
 */
public class PointShape extends Shape {
    public Vec2 m_localPosition;
    public float m_mass;

    public PointShape(ShapeDef def) {
    	super(def);
    	assert(def.type == ShapeType.POINT_SHAPE);
    	PointDef pointDef = (PointDef)def;
    	m_type = ShapeType.POINT_SHAPE;
    	m_localPosition = pointDef.localPosition.clone();
    	m_mass = pointDef.mass;
    }
    
    // DMNOTE optimized
	@Override
	public void computeAABB(AABB aabb, XForm transform) {
		//Vec2 p = transform.position.add(Mat22.mul(transform.R, m_localPosition));
    	Vec2 p = new Vec2();
    	Mat22.mulToOut(transform.R, m_localPosition, p);
    	p.add(transform.position);
    	aabb.lowerBound.set(p.x-Settings.EPSILON, p.y-Settings.EPSILON);
    	aabb.upperBound.set(p.x+Settings.EPSILON, p.y+Settings.EPSILON);
	}

	@Override
	public void computeMass(MassData massData) {
		massData.mass = m_mass;
    	massData.center = m_localPosition.clone();
    	massData.I = 0.0f;
	}

	// DMNOTE pooled
	private Vec2 sweptP1 = new Vec2();
	private Vec2 sweptP2 = new Vec2();
	// DMNOTE fairly hot method, called every update
	@Override
	public void computeSweptAABB(AABB aabb, XForm transform1, XForm transform2) {
		
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

	@Override
	public boolean testPoint(XForm xf, Vec2 p) {
		// DMNOTE could use more optimization
		return false;
	}

	// DMNOTE optimized
	@Override
	public void updateSweepRadius(Vec2 center) {
		//Vec2 d = m_localPosition.sub(center);
		float dx = m_localPosition.x - center.x;
		float dy = m_localPosition.y - center.y;
    	m_sweepRadius = (float)Math.sqrt(dx*dx + dy*dy) - Settings.toiSlop;
	}
	
	//Returns a copy of local position
    public Vec2 getLocalPosition() {
    	return m_localPosition.clone();
    }

    public float getMass() {
    	return m_mass;
    }

}
