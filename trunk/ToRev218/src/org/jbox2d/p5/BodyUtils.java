package org.jbox2d.p5;

//import java.util.ArrayList;
//
//import org.jbox2d.collision.AABB;
//import org.jbox2d.collision.CircleDef;
//import org.jbox2d.collision.CircleShape;
//import org.jbox2d.collision.PolygonDef;
//import org.jbox2d.collision.PolygonShape;
//import org.jbox2d.collision.Shape;
//import org.jbox2d.collision.ShapeDef;
//import org.jbox2d.collision.ShapeType;
//import org.jbox2d.common.Vec2;
//import org.jbox2d.dynamics.Body;
//import org.jbox2d.dynamics.BodyDef;
//import org.jbox2d.dynamics.World;
//import org.jbox2d.dynamics.joints.RevoluteJointDef;
//

// Currently inactive - will be pulled in and enabled as needed

/**
 * Set of static methods for creating more complex bodies easily.
 */
public class BodyUtils
{
//	
//	public static ArrayList<Body> createRope(World world, Vec2 a, Vec2 b,
//			float segmentThickness, int segments)
//	{
//		Vec2 diff = b.sub(a);
//		float length = diff.normalize();
//		
//		float segLength = length / segments;
//		
//		ArrayList<Body> bodyList = new ArrayList<Body>();
//	
//		Body lastBody = null;
//		for (int i = 0; i < segments; i++)
//		{
//			Vec2 prev = new Vec2(a.x+diff.x*segLength*i, a.y+diff.y*segLength*i);
//			Vec2 next = new Vec2(a.x+diff.x*segLength*(i+1), a.y+diff.y*segLength*(i+1));
//			Body body = createRoundStick(world, prev, next, segmentThickness);
//			body.setBullet(false);
//			
//			if (lastBody != null)
//			{
//				/*
//				 * Join together this and the last one.
//				 */
//				RevoluteJointDef jd = new RevoluteJointDef();
//				jd.body1 = lastBody;
//				jd.body2 = body;
//				jd.localAnchor1 = lastBody.getLocalPoint(prev);
//				jd.localAnchor2 = body.getLocalPoint(prev);
//				jd.enableMotor = true;
//				jd.maxMotorTorque = body.getMass()*10.0f;
//				jd.motorSpeed = 0;
//				jd.collideConnected = false;
//				world.createJoint(jd);
//			}
//			bodyList.add(body);
//			lastBody = body;
//		}
//	
//		return bodyList;
//	}
//	
//	//TODO: fix createBridge, it's placing things wrong (seems to start at a, but not end at b)
//	//[Done, I think - e]
//	/**
//	 * Creates a bridge between two points. Uses round sticks as the segments.
//	 * @param a The first anchor point
//	 * @param b The second anchor point
//	 * @param maxSegmentLength The maximum length allowed for a segment. For obvious reasons, sometimes segments will need to be smaller than the given length in order to evenly divide the desired bridge length.
//	 * @param segmentThickness The thickness for each segment.
//	 * @param sagginess The ratio of bridge length to the distance between the anchor points. Must be equal to or greater than 1. 
//	 * <!-- @param jointDefs An optional array of jointDefs to be used when joining the bridge segments together (may be left null). For example, you may want to use an elastic distance joint, or a distance joint combined with a damped rotation joint that makes the bridge "stiffer" --> 
//	 * @return ArrayList of bodies created
//	 */
//	public static ArrayList<Body> createBridge(World world, Vec2 a, Vec2 b,
//			float maxSegmentLength, float segmentThickness, float sagginess)
//	{
//		Vec2 diff = b.sub(a);
//		float length = diff.length();
//		Vec2 middle = a.add(diff.mul(.5f));
//		Vec2 normal = new Vec2(diff.y, diff.x); // Create a vector rotated 90 degrees to the diff vector.
//		normal.normalize();
//
//		/*
//		 * For creating the segments we will follow a V-shaped path, whose height is defined by the "sagginess" parameter.
//		 */
//		// Use some pythagorean theorem to decide how far down the bottom point of the V needs to be.
//		float sagLength = sagginess * length;
//		float height = (float) Math.sqrt(sagLength * sagLength - length
//				* length);
//		Vec2 bottomPoint = middle.add(normal.mul(height)); // The bottom point of our V.
//
//		/*
//		 * Figure out how many segments we need to make, using the maxSegmentLength.
//		 */
//		maxSegmentLength = Math.min(sagLength, maxSegmentLength); // Sanity check.
//		float numSegments = sagLength / maxSegmentLength; // Find the ideal, but fractional, number of segments
//		numSegments = (int) Math.ceil(numSegments); // Round UP so that we end up creating segments that are smaller than the given max segment length.
//		if (numSegments % 2 != 0)
//			numSegments++;
//		float segmentLength = sagLength / numSegments;
//
//		ArrayList<Body> bodyList = new ArrayList<Body>();
//
//		Body lastBody = null;
//		for (int i = 0; i < numSegments - 1; i++)
//		{
//			Vec2 prev = null;
//			Vec2 next = null;
//			Vec2 full = bottomPoint.sub(a);
//			prev = a.add(full.mul(i * segmentLength));
//			next = a.add(full.mul((i + 1) * segmentLength));
//			Body body = createRoundStick(world, prev, next, segmentThickness);
//			body.setBullet(true);
//			if (i == 0 || i == numSegments - 2)
//			{
//				RevoluteJointDef rj = new RevoluteJointDef();
//				rj.body1 = world.getGroundBody();
//				rj.body2 = body;
//				Vec2 point = next;
//				if (i == 0)
//					point = prev;
//				rj.localAnchor1 = rj.body1.getLocalPoint(point);
//				rj.localAnchor2 = rj.body2.getLocalPoint(point);
//				world.createJoint(rj);
//			}
//			if (lastBody != null)
//			{
//				/*
//				 * Join together this and the last one.
//				 */
//				RevoluteJointDef jd = new RevoluteJointDef();
//				jd.body1 = lastBody;
//				jd.body2 = body;
//				jd.localAnchor1 = lastBody.getLocalPoint(prev);
//				jd.localAnchor2 = body.getLocalPoint(prev);
//				jd.enableMotor = true;
//				jd.maxMotorTorque = body.getMass();
//				jd.motorSpeed = 0;
//				jd.collideConnected = false;
//				world.createJoint(jd);
//			}
//			bodyList.add(body);
//			lastBody = body;
//		}
//
//		return bodyList;
//	}
//
//	//TODO: this properly belongs somewhere other than BodyUtils, I think...
//	/**
//	 * Extract a ShapeDef from a Shape so that you can, for instance,
//	 * copy a shape or recreate it on a new body.
//	 * <p>
//	 * You will need to downcast the ShapeDef to the appropriate
//	 * type - use ShapeDef::getType to see what type is appropriate.
//	 * </p>
//	 * <p>
//	 * Watch out when moving shapes from one body to another - the local
//	 * positions of shapes are relative to the body centers, so you will
//	 * need to compensate for that if you don't want the shape to jump
//	 * from one place to another.
//	 * @param s Shape to extract definition from
//	 * @return A ShapeDef that, if used via Body::createShape(shapeDef),
//	 * should result in exactly the same shape showing up on the body as
//	 * was passed as the parameter to this method.  Note that the Body mass
//	 * would still need to be set appropriately, as this information lives
//	 * on the body, not the shape.
//	 */
//	public static ShapeDef shapeToShapeDef(Shape s) {
//		ShapeType type = s.getType();
//		if (type == ShapeType.CIRCLE_SHAPE) {
//			CircleDef cd = new CircleDef();
//			CircleShape cs = (CircleShape)s;
//			cd.filter.categoryBits = cs.m_filter.categoryBits;
//			cd.filter.maskBits = cs.m_filter.maskBits;
//			cd.filter.groupIndex = cs.m_filter.groupIndex;
//			cd.density = cs.m_density;
//			cd.friction = cs.m_friction;
//			cd.isSensor = cs.isSensor();
//			cd.restitution = cs.m_restitution;
//			cd.radius = cs.getRadius();
//			cd.userData = cs.getUserData();
//			cd.localPosition = cs.getLocalPosition();
//			return cd;
//		} else if (type == ShapeType.POLYGON_SHAPE) {
//			PolygonDef pd = new PolygonDef();
//			PolygonShape ps = (PolygonShape)s;
//			pd.filter.categoryBits = ps.m_filter.categoryBits;
//			pd.filter.maskBits = ps.m_filter.maskBits;
//			pd.filter.groupIndex = ps.m_filter.groupIndex;
//			pd.density = ps.m_density;
//			pd.friction = ps.m_friction;
//			pd.isSensor = ps.isSensor();
//			pd.restitution = ps.m_restitution;
//			pd.userData = ps.getUserData();
//			int nVerts = ps.getVertexCount();
//			Vec2[] vertices = ps.getVertices();
//			for (int i=0; i<nVerts; ++i) {
//				pd.vertices.add(vertices[i]);
//			}
//			return pd;
//		}
//		return null;
//	}
//
//	public static Body createRoundStick(World world, Vec2 a, Vec2 b, float r)
//	{
//		BodyDef bd = PhysicsUtils.newBodyDef();
//
//		/*
//		 * Convert the anchor points into a center coordinate, width, and angle.
//		 */
//		Vec2 diff = b.sub(a);
//		float angle = PhysicsUtils.angle(diff);
//		float length = diff.length() + 2 * r; // Length of desired stick.
//
//		Vec2 center = a.add(diff.mul(.5f)); // Add half the difference to A, to get the center coords.
//
//		//ewj TODO: fix this, the createRoundRectangle code actually creates four circles,
//		//whereas you only actually need two.  Redundant shapes can cause trouble in the
//		//solver, both performance-wise and results-wise, since you're double-solving what
//		//should be a single contact.
//		//Better: fix up createRoundRectangle so it never does more work than it has to!
//		/*
//		 * Construct a call to createRoundRectangle, to avoid needless code duplication.
//		 */
//		Body body = createRoundRectangle(world, center.x, center.y, length,
//				r * 2, r);
//
//		/*
//		 * Rotate the body.
//		 */
//		body.setXForm(body.getPosition(), angle);
//		//		body.m_xf.R.setAngle((float)Math.PI);
//
//		return body;
//	}
//
//    public static Body createRoundRectangle(World world, float cx, float cy,
//            float w, float h, float r, Material m) {
//        Body b = BodyUtils.createRoundRectangle(world,cx,cy,w,h,r);
//        b.setMassFromShapes();
//        m.applyAll(b);
//        return b;
//    }
//
//    /**
//	 * Creates a rectangular body with rounded corners.
//	 *
//	 * Is actually composed of five rectangles and four (massless) circles.
//	 *
//	 * @param world the World in which to create the body.
//	 * @param cx x coordinate of the center of the rectangle
//	 * @param cy y coordinate of the center of the rectangle
//	 * @param w the width of the rectangle
//	 * @param h the height of the rectangle
//	 * @param r the radius of the rounded corners
//	 * @param m a Material object to apply (optional)
//	 * @return a new rounded rectangular Body
//	 */
//	public static Body createRoundRectangle(World world, float cx, float cy,
//			float w, float h, float r)
//	{
//		BodyDef bd = PhysicsUtils.newBodyDef();
//		bd.position.set(cx, cy);
//		Body b = world.createBody(bd);
//
//		/*
//		 * The width and height specified in the parameters is the "total" width (including rounded corners),
//		 * so let's store the "inner" widths for convenience.
//		 */
//		float innerWidth = w - 2 * r;
//		float innerHeight = h - 2 * r;
//
//		if (innerWidth == 0 && innerHeight == 0)
//		{
//			/*
//			 * Edge case -- you are actually defining a circle!
//			 */
//			return createCircle(world, cx, cy, r);
//		} else if (r == 0)
//		{
//			/*
//			 * Edge case -- you are actually defining a rectangle!
//			 */
//			return createBox(world, cx, cy, w, h);
//		} else if (innerWidth < 0 || innerHeight < 0)
//		{
//			throw new IllegalArgumentException(
//					"Error: Radius is larger than half the width or height -- this doesn't make sense for a rounded rectangle!");
//		}
//
//		PolygonDef pd;
//
//		if (innerWidth > 0 && innerHeight > 0)
//		{
//			// Inner rectangle.
//			pd = new PolygonDef();
//			setAsBox(pd, innerWidth, innerHeight, new Vec2(0, 0), 0);
//			b.createShape(pd);
//		}
//
//		if (innerWidth > 0)
//		{
//			// Lower rectangle.
//			pd = new PolygonDef();
//			setAsBox(pd, innerWidth, r, new Vec2(0, -innerHeight / 2 - r / 2),
//					0);
//			b.createShape(pd);
//
//			// Upper rectangle.
//			pd = new PolygonDef();
//			setAsBox(pd, innerWidth, r, new Vec2(0, +innerHeight / 2 + r / 2),
//					0);
//			b.createShape(pd);
//		}
//
//		if (innerHeight > 0)
//		{
//			// Left rectangle.
//			pd = new PolygonDef();
//			setAsBox(pd, r, innerHeight, new Vec2(-innerWidth / 2 - r / 2, 0),
//					0);
//			b.createShape(pd);
//
//			// Right rectangle.
//			pd = new PolygonDef();
//			setAsBox(pd, r, innerHeight, new Vec2(innerWidth / 2 + r / 2, 0), 0);
//			b.createShape(pd);
//		}
//
//		/*
//		 * Now, let's create the circles to "round out" the body.
//		 */
//		CircleDef cd = new CircleDef();
//		cd.radius = r;
//
//		cd.localPosition.set(-innerWidth / 2, innerHeight / 2); // Top left.
//		b.createShape(cd);
//		cd.localPosition.set(innerWidth / 2, innerHeight / 2); // Top right.
//		b.createShape(cd);
//		cd.localPosition.set(-innerWidth / 2, -innerHeight / 2); // Bottom left.
//		b.createShape(cd);
//		cd.localPosition.set(innerWidth / 2, -innerHeight / 2); // Bottom right.
//		b.createShape(cd);
//
//		return b;
//	}
//
//	/**
//	 * Creates a "default" circle with the given xy center, radius, and Material (if applicable).
//	 * @param w
//	 * @param x
//	 * @param y
//	 * @param r
//	 * @param m a Material object (may be null)
//	 * @return a new circular body
//	 */
//	public static Body createCircle(World w, float x, float y, float r)
//	{
//		BodyDef bd = PhysicsUtils.newBodyDef();
//		bd.position.set(x, y);
//		CircleDef c = new CircleDef();
//		c.localPosition.set(0, 0);
//		c.radius = r;
//		Body cb = w.createBody(bd);
//		cb.createShape(c);
//		return cb;
//	}
//
//    /**
//     * Creates a clone of the existing body at the specified location.
//     * @param x loc
//     * @param y loc
//     * @param b body to clone
//     * @return clone of body at the specified location.
//     */
//    public static Body cloneBody(float x, float y, Body b) {
//        World w = b.getWorld();
//        BodyDef bd = PhysicsUtils.newBodyDef();
//        bd.position.set(x, y);
//        ShapeDef shapeDef = BodyUtils.shapeToShapeDef(b.getShapeList());
//        Body cb = w.createBody(bd);
//        cb.createShape(shapeDef);
//        return cb;
//    }
//
//    /**
//	 * Creates a hollow box. Returns the massless body -- if you want it to be dynamic,
//	 * just add some weight! We recommend using a predefined Material object and calling the applyAll(Body b) method.
//	 *
//	 * @param world
//	 * @param cx
//	 * @param cy
//	 * @param w
//	 * @param h
//	 * @param thickness
//	 * @return
//	 */
//	public static Body createHollowBox(World world, float cx, float cy,
//			float w, float h, float thickness, boolean dynamic)
//	{
//		Body b;
//		b = world.createBody(PhysicsUtils.newBodyDef());
//		PolygonDef pd = new PolygonDef();
//		/*
//		 * Note that we have to use w/2 and thickness/2 in the first two arguments here,
//		 * because of a bug in the box2D code in which the hx and hy parameters are interpreted as
//		 * HALF the height and width of the desired box, not the full width.
//		 */
//		setAsBox(pd, w, thickness, new Vec2(cx, cy + h / 2 - thickness / 2), 0); // Bottom piece.
//		b.createShape(pd);
//		setAsBox(pd, w, thickness, new Vec2(cx, cy - h / 2 + thickness / 2), 0); // Top piece.
//		b.createShape(pd);
//
//		setAsBox(pd, thickness, h, new Vec2(cx - w / 2 + thickness / 2, cy), 0); // Left piece.
//		b.createShape(pd);
//		setAsBox(pd, thickness, h, new Vec2(cx + w / 2 - thickness / 2, cy), 0); // Right piece.
//		b.createShape(pd);
//		if (dynamic)
//		{
//			b.setMassFromShapes();
//			Material.DEAD_WEIGHT().applyAll(b);
//		}
//		return b;
//	}
//
//	public static Body createHollowBox(World world, float cx, float cy,
//			float w, float h, float thickness)
//	{
//		return createHollowBox(world, cx, cy, w, h, thickness, false);
//	}
//
//	/**
//	 * Create a physical world boundary.  This will ensure that your
//	 * objects cannot leave the world AABB (which would usually result
//	 * in freezing).
//	 * @param world
//	 * @param thickness
//	 * @return
//	 */
//	public static Body createBoundingBox(NewWorld world, float thickness)
//	{
//		AABB aabb = world.getAABB();
//
//		Vec2 lo = aabb.lowerBound;
//		Vec2 hi = aabb.upperBound;
//		Vec2 diff = hi.sub(lo);
//		Vec2 center = lo.add(diff.mul(.5f)); // Find the center coordinates. Yeah, I'm lazy.
//
//		return createHollowBox(world, center.x, center.y, diff.x, diff.y,
//				thickness, false);
//	}
//
//    public static Body createBox(World world, float cx, float cy, float w,
//            float h, Material m) {
//        Body b = BodyUtils.createBox(world,cx,cy,w,h);
//        b.setMassFromShapes();
//        m.applyAll(b);
//        return b;
//    }
//
//    public static Body createBox(World world, float cx, float cy, float w,
//			float h)
//	{
//		BodyDef bd = PhysicsUtils.newBodyDef();
//		Body b = world.createBody(bd);
//        b.setBullet(false);
//        PolygonDef pd = new PolygonDef();
//		setAsBox(pd, w, h, new Vec2(cx, cy), 0);
//		b.createShape(pd);
//		return b;
//	}
//
//	//ewj note: the setAsBox thing is officially not a bug in Box2d, since the behavior
//	//is documented (it takes half-widths/heights, and they are listed as such),
//	//and it's unlikely to change since it's been done that way for several versions.
//	/**
//	 * A wrapper method that fixes the JBox2d setAsBox bug. Included here so that if Eric or Erin Catto ever agree
//	 * to fix the bug, we won't have to change a ton of code to compensate, just this little guy.
//	 * @param pd
//	 * @param width
//	 * @param height
//	 * @param center
//	 * @param angle
//	 */
//	static void setAsBox(PolygonDef pd, float width, float height,
//			Vec2 center, float angle)
//	{
//		pd.setAsBox(width / 2, height / 2, center, angle);
//	}

}
