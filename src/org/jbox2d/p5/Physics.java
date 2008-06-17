package org.jbox2d.p5;

import java.lang.reflect.Method;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.collision.ShapeDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.DistanceJoint;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.GearJoint;
import org.jbox2d.dynamics.joints.GearJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.JointType;
import org.jbox2d.dynamics.joints.PrismaticJoint;
import org.jbox2d.dynamics.joints.PrismaticJointDef;
import org.jbox2d.dynamics.joints.PulleyJoint;
import org.jbox2d.dynamics.joints.PulleyJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.testbed.ProcessingDebugDraw;
import org.jbox2d.testbed.TestSettings;

import processing.core.PApplet;

/**
 * A wrapper class to make using JBox2d with Processing dead simple.
 * <BR><BR>
 * The basics...
 * <BR><BR>
 * Required import:
 * <pre>
 * import org.jbox2d.p5.*;</pre>
 * Optional imports (if you're doing anything substantial with the
 * physical objects, you'll want some or all of these), ordered by
 * the likelihood of your needing them:
 * <pre>
 * import org.jbox2d.common.*;
 * import org.jbox2d.dynamics.*;
 * import org.jbox2d.dynamics.joints.*;
 * import org.jbox2d.collision.*;
 * import org.jbox2d.dynamics.contacts.*;</pre>
 * The above files make up the full JBox2d library.  If you're worried
 * about importing too much stuff, you're welcome to figure out exactly 
 * what you do and don't need, but there are way too many class files 
 * to go through individually here.
 * <BR><BR>
 * It is recommended that you run your sketch at 60 fps, as that is the
 * speed that physics is simulated at.  The physics simulation is <em>not</em>
 * connected to the sketch frame rate, so even if your sketch runs at 30 fps,
 * the physics will still advance by 1/60 of a second every frame.  You can
 * change the time step by doing physics.getSettings().hz = newFrequency; but
 * bear in mind that this engine is only properly tuned for 60 hz, so you might
 * get instability or poor quality if you do so.
 * <BR><BR>
 * To create a world fitted to your sketch size, assuming that sketchWidth
 * and sketchHeight are both set to the proper values AND the sketch has
 * completely initialized (threading bugs in some recent Processing versions
 * have caused proper size initialization to be delayed until a couple of runs
 * into the draw loop - if the JBox2d drawer seems to be initialized to the
 * wrong size, you probably need to delay this call until a few frames in to
 * the sketch run), do the following:
 * <pre>
 * Physics physics = new Physics(this, sketchWidth, sketchHeight);</pre>
 * You do <em>not</em> need to add anything special to your draw loop to
 * cause the physics to be simulated and/or rendered.  This is done automatically.
 * <BR><BR>
 * The default setup (as above) sets up a scaling factor so that 10 pixels = 1 meter.
 * Most functions in this wrapper use pixel units, so you may position things
 * naturally in screen space.  However, Processing transformations (scale, translate,
 * rotate) are not taken into account.
 * <BR><BR>
 * If you need to do something in world coordinates (for instance, the gravity vector
 * in the alternate constructor is specified in meters, not pixels), you can use
 * the worldToScreen and screenToWorld functions to convert back and forth.
 * <BR><BR>
 * To destroy a world:
 * <pre>
 * physics.destroy();</pre>
 * To create a static (immobile) body:
 * <pre>
 * physics.setDensity(0.0f);
 * physics.createRect(x0, y0, x1, y1);</pre>
 * To create a normal moving body:
 * <pre>
 * physics.setDensity(someNonZeroDensity);
 * physics.createRect(x0, y0, x1, y1);</pre>
 * You can also setFriction(float) and setRestitution(float) to change
 * the way your objects collide.  This wrapper retains the last set values,
 * much like Processing does with stroke and fill.
 * <BR><BR>
 * Of course there are more features (joints, sensors, continuous collision detection, custom
 * rendering routines, and more to come!), but for that, please see the rest of the documentation.  Enjoy!
 * <BR><BR>
 * @author ewjordan
 */
public class Physics {
	// All the crap you'd otherwise have to handle yourself
	private World m_world;
	private PApplet m_parent;
	private ProcessingDebugDraw m_draw;
	private Vec2 m_gravity;
	private AABB m_worldAABB;
	private TestSettings m_settings;
	
	private Body[] m_border;
	
	// Body creation settings
	private float m_density;
	private float m_restitution;
	private float m_friction;
	private boolean m_bullet;
	private boolean m_sensor;
	
	private Method m_customRenderingMethod;
	private Object m_customRenderingObject;
	
	
	/**
	 * Set up a default physics world.
	 * This sets a world bounding box at twice
	 * the screen distance from the center, and
	 * a hard boundary at the window edges.
	 * Uses a default scaling where 1 m = 10 px.
	 * @param parent
	 */
	public Physics(PApplet parent, float screenW, float screenH) {
		this(	parent,									//parent PApplet
				screenW, screenH,
				0.0f,-10.0f, 							//gravity vector
				//0.5f*parent.width, 0.5f*parent.height, 	//center of the world
				2*screenW,2*screenH, 		//AABB (px)
				screenW,screenH, 			//hard border (px)
				10.0f); 								//scaling factor, pixels per meter
	}
	
	/**
	 * Set up a physics world.
	 * 
	 * @param parent The PApplet this physics world should use
	 * @param gravX The x component of gravity, in meters/sec^2
	 * @param gravY The y component of gravity, in meters/sec^2
	 * @param screenAABBWidth The world's width, in pixels - should be significantly larger than the area you intend to use
	 * @param screenAABBHeight The world's height, in pixels - should be significantly larger than the area you intend to use
	 * @param borderBoxWidth The containing box's width - should be smaller than the world width, so that no object can escape
	 * @param borderBoxHeight The containing box's height - should be smaller than the world height, so that no object can escape
	 * @param pixelsPerMeter Pixels per physical meter
	 */
	public Physics(PApplet parent, float screenW, float screenH,
								   float gravX, float gravY,
								   float screenAABBWidth, float screenAABBHeight,
								   float borderBoxWidth, float borderBoxHeight,
								   float pixelsPerMeter) {
		m_parent = parent;
		m_draw = new ProcessingDebugDraw(m_parent);
		m_gravity = new Vec2(gravX, gravY);
		m_draw.setCamera(0, 0, pixelsPerMeter);
		Vec2 minWorldAABB = new Vec2(-screenAABBWidth*.5f/pixelsPerMeter, -screenAABBHeight*.5f/pixelsPerMeter);
		Vec2 maxWorldAABB = minWorldAABB.mul(-1.0f);
		boolean doSleep = true;
		m_world = new World(new AABB(minWorldAABB,maxWorldAABB),m_gravity,doSleep);
		m_world.setDebugDraw(m_draw);
		m_world.setDrawDebugData(false);
		m_settings = new TestSettings();
		
		setDensity(0.0f);
		setRestitution(0.1f);
		setFriction(0.4f);
		setBullet(false);
		setSensor(false);
		
		m_border = createHollowBox(screenW*.5f, screenH*.5f, borderBoxWidth, borderBoxHeight, 10.0f);
		
		parent.registerDraw(this);
	}
	
	/**
	 * Called automatically by Processing.
	 */
	public void draw() {
		m_world.setWarmStarting(m_settings.enableWarmStarting);
		m_world.setPositionCorrection(m_settings.enablePositionCorrection);
		m_world.setContinuousPhysics(m_settings.enableTOI);
		
		m_world.step(1.0f/m_settings.hz, m_settings.iterationCount);
		
		if (m_customRenderingMethod == null) {
			defaultDraw(m_world);
		} else {
			try{
				m_customRenderingMethod.invoke(m_customRenderingObject, m_world);
			} catch (Exception e) {
				m_parent.die("Error invoking custom rendering method.", e);
			}
		}

	}
	
	/**
	 * For advanced users only.
	 * <BR><BR>
	 * Set a custom rendering method to be called.  Use this
	 * if the default renderer is not drawing things the way
	 * you like, or if you need more flexibility.  This
	 * will allow a very modular approach to rendering, whereby
	 * you can change the entire graphical style on the fly by
	 * switching the render function.  Hopefully some people a
	 * lot more graphically skilled than I am will provide some
	 * cool looking functions for this purpose!
	 * <BR><BR>
	 * The method is set through Java's reflection API, so you
	 * may call any method that takes a World object as a parameter.
	 * The usual Java way is to force implementation of an interace,
	 * but this way you can write things in a simpler manner, within
	 * the PDE and without multiple tabs or pure Java.
	 * Just pass the object that has the method along with the name
	 * of the method.
	 * <BR><BR>
	 * e.g. if you have the following method defined in your
	 * sketch:
	 * <pre>
	 * void myDrawMethod(World world) {
	 *   // Do a bunch of stuff
	 * }
	 * </pre>
	 * then inside either the draw or setup functions you could write:
	 * <pre>
	 * setCustomRenderingMethod(this, "myDrawMethod");</pre>
	 * to register that function.  Use unsetCustomRenderingMethod() to
	 * go back to the default renderer.
	 * <BR><BR>
	 * If you're interested in writing your own renderer, you'll likely need
	 * to look at the source code, which you can get to from http://www.jbox2d.org.
	 * In particular, the org.jbox2d.dynamics.World file has a drawDebugData() 
	 * function, which, after some preprocessing, makes some calls out to 
	 * org.jbox2d.testbed.ProcessingDebugDraw to do the actual drawing.  That
	 * should give you a place to start from, at least.
	 * <BR><BR>
	 * Note also that this rendering method has nothing to do with
	 * Java2d vs. P3D vs. OpenGL - that choice is made at the
	 * beginning of your sketch, and this library won't let you change it!
	 * This functionality merely relates to the way bodies and shapes
	 * are translated into drawing calls; the drawing calls themselves
	 * rely on whatever renderer you chose in your size() function.
	 * <BR><BR>
	 * @param object The object in which your method is defined ('this' should work if
	 * the object is defined in a .pde file and not within a class)
	 * @param methodName The name of the method (without the parenthesis) to call
	 */
	public void setCustomRenderingMethod(Object object, String methodName) {
		Class c = object.getClass();
	    try {
	      Class methodArgs[] = new Class[] { World.class };
	      Method method = c.getMethod(methodName, methodArgs);
	      m_customRenderingObject = object;
	      m_customRenderingMethod = method;
	    } catch (Exception e) {
	      m_parent.die("Could not register " + methodName + "(World) for " + object + 
	    		  ", make sure the method name is spelled correctly and that the method " +
	    		  "takes a World object as an argument!", e);
	    }
	}
	
	/**
	 * Clear any custom rendering method that has been set, and
	 * revert to the default Box2d debug renderer.
	 */
	public void unsetCustomRenderingMethod() {
		m_customRenderingMethod = null;
		m_customRenderingObject = null;
	}
	
	/**
	 * Draws the scene using the default render options.
	 * Automatically called by the engine unless you have
	 * specified your own rendering routine.
	 */
	public void defaultDraw(World world) {
		m_draw.setFlags(0);
		if (m_settings.drawShapes) m_draw.appendFlags(DebugDraw.e_shapeBit);
		if (m_settings.drawJoints) m_draw.appendFlags(DebugDraw.e_jointBit);
		if (m_settings.drawCoreShapes) m_draw.appendFlags(DebugDraw.e_coreShapeBit);
		if (m_settings.drawAABBs) m_draw.appendFlags(DebugDraw.e_aabbBit);
		if (m_settings.drawOBBs) m_draw.appendFlags(DebugDraw.e_obbBit);
		if (m_settings.drawPairs) m_draw.appendFlags(DebugDraw.e_pairBit);
		if (m_settings.drawCOMs) m_draw.appendFlags(DebugDraw.e_centerOfMassBit);

		world.setDrawDebugData(true);
		world.drawDebugData();
		world.setDrawDebugData(false);
	}
	
	/**
	 * Get an editable copy of the current TestSettings so
	 * that you may change certain aspects of the simulation
	 * and display.  You do not need to re-set anything after
	 * editing these settings, the changes take effect immediately.
	 * <BR><BR>
	 * The list of useful fields in the TestSettings objects follows:
	 * <pre>
	 * public int hz; // "frame" rate of physics simulation - best to leave at 60
     * public int iterationCount; // number of constraint iterations - set to 10 normally
	 * public boolean enableWarmStarting; // makes constraints work better by reusing last results
     * public boolean enablePositionCorrection; // leave this on...without it, things turn to mush
     * public boolean enableTOI; // enable/disable continuous collision detection
	 * public boolean drawShapes;
	 * public boolean drawJoints;
	 * public boolean drawCoreShapes;
	 * public boolean drawOBBs;
	 * public boolean drawCOMs;
     * public boolean drawImpulses;
     * public boolean drawAABBs;
     * public boolean drawPairs;
	 * public boolean drawContactPoints;
	 * public boolean drawContactNormals;
	 * public boolean drawContactForces;
	 * public boolean drawFrictionForces;
	 * </pre>
	 * Note: the drawing settings only affect the default debug renderer.
	 * If you have specified your own renderer, you will have to manually
	 * read off and apply these settings if you wish to use them.
	 * @return A reference to the active TestSettings object
	 */
	public TestSettings getSettings() {
		return m_settings;
	}
	
	/**
	 * Create a hollow box of the given screen dimensions.
	 * @param centerX Center of box x coordinate (in screen coordinates)
	 * @param centerY Center of box y coordinate (in screen coordinates)
	 * @param width Width of box (screen scale)
	 * @param height Height of box (screen scale)
	 * @param thickness Thickness of box edge (screen scale)
	 * @return
	 */
	public Body[] createHollowBox(float centerX, float centerY, float width, float height, float thickness) {
		Body[] result = new Body[4];
		result[0] = createRect(centerX - width*.5f - thickness*.5f, centerY - height*.5f - thickness*.5f,
				   			   centerX - width*.5f + thickness*.5f, centerY + height*.5f + thickness*.5f);
		result[1] = createRect(centerX + width*.5f - thickness*.5f, centerY - height*.5f - thickness*.5f,
				   			   centerX + width*.5f + thickness*.5f, centerY + height*.5f + thickness*.5f);
		result[2] = createRect(centerX - width*.5f - thickness*.5f, centerY + height*.5f - thickness*.5f,
				   			   centerX + width*.5f + thickness*.5f, centerY + height*.5f + thickness*.5f);
		result[3] = createRect(centerX - width*.5f - thickness*.5f, centerY - height*.5f - thickness*.5f,
				   			   centerX + width*.5f + thickness*.5f, centerY - height*.5f + thickness*.5f);
		return result;
	}
	
	/**
	 * Create a rectangle given by screen coordinates of corners.
	 * @param x0
	 * @param y0
	 * @param x1
	 * @param y1
	 * @return
	 */
	public Body createRect(float x0, float y0, float x1, float y1) {
		float cxs = (x0 + x1) * .5f;
		float cys = (y0 + y1) * .5f;
		float wxs = Math.abs(x1-x0);
		float wys = Math.abs(y1-y0);
		//System.out.println("Screen: ("+cxs + ","+cys+")");
		Vec2 center = m_draw.screenToWorld(cxs, cys);
		//System.out.println("World: "+center);
		float halfWidthWorld = .5f*m_draw.screenToWorld(wxs);
		float halfHeightWorld = .5f*m_draw.screenToWorld(wys);
		//System.out.println("Half Width world: "+halfWidthWorld);
		PolygonDef pd = new PolygonDef();
		pd.setAsBox(halfWidthWorld, halfHeightWorld);
		setShapeDefProperties(pd);
		
		BodyDef bd = new BodyDef();
		setBodyDefProperties(bd);
		
		Body b = m_world.createBody(bd);
		b.createShape(pd);
		if (m_density > 0.0f) b.setMassFromShapes();
		
		b.setXForm(center, 0.0f);
		
		return b;
	}
	
	/**
	 * Create a circle in screen coordinates
	 * @param x
	 * @param y
	 * @param r
	 * @return
	 */
	public Body createCircle(float x, float y, float r) {
		Vec2 center = m_draw.screenToWorld(x,y);
		float rad = m_draw.screenToWorld(r);
		
		CircleDef cd = new CircleDef();
		cd.radius = rad;
		setShapeDefProperties(cd);
		
		BodyDef bd = new BodyDef();
		setBodyDefProperties(bd);
		
		Body b = m_world.createBody(bd);
		b.createShape(cd);
		if (m_density > 0.0f) b.setMassFromShapes();
		
		b.setXForm(center, 0.0f);
		
		return b;
	}
	
	/**
	 * Create a polygon based on vertices.
	 * <BR><BR>
	 * Polygons must be:
	 * <ul>
	 * <li>Ordered clockwise in screen coordinates (which
	 * becomes counterclockwise in world coordinates).
	 * <li>Non self-intersecting.
	 * <li>Convex
	 * </ul>
	 * Failure to adhere to any of these restrictions may cause
	 * simulation crashes or problems.  In particular, if your
	 * objects are showing up as static objects instead of dynamic
	 * ones, and are not colliding correctly, you have probably 
	 * not met the clockwise ordering requirement.
	 * <BR><BR>
	 * This can be called with any number of vertices passed as
	 * pairs of interleaved floats, for instance:
	 * <pre>
	 * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3);</pre>
	 * or
	 * <pre>
	 * createPolygon(x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5);</pre>
	 * or
	 * <pre>
	 * float[] xyInterleaved = {x0,y0,x1,y1,x2,y2,x3,y3,x4,y4};
	 * createPolygon(xyInterleaved);</pre>
	 * are all fine.
	 * @param vertices Any number of pairs of x,y floats, or an array of the same (screen coordinates)
	 * @return
	 */
	public Body createPolygon(float... vertices) {
		if (vertices.length % 2 != 0) 
			throw new IllegalArgumentException("Vertices must be given as pairs of x,y coordinates, " +
											   "but number of passed parameters was odd.");
		int nVertices = vertices.length / 2;
		PolygonDef pd = new PolygonDef();
		for (int i=0; i<nVertices; ++i) {
			Vec2 v = screenToWorld(vertices[2*i],vertices[2*i+1]);
			pd.addVertex(v);
		}
		setShapeDefProperties(pd);
		
		BodyDef bd = new BodyDef();
		setBodyDefProperties(bd);
		
		Body b = m_world.createBody(bd);
		b.createShape(pd);
		if (m_density > 0.0f) b.setMassFromShapes();
		
		return b;
	}
	
	/**
	 * Create a distance (stick) joint between two bodies
	 * that holds the specified points at a constant distance.
	 * <BR><BR>
	 * Once the distance joint is created, it may be turned into
	 * a "soft" distance joint by using DistanceJoint::setFrequencyHz(float)
	 * to set the frequency to a non-zero value, and using 
	 * DistanceJoint::setDampingRatio(float) to tune the damping constant.
	 * <BR><BR>
	 * Distance joints do not support joint limits or motors.
	 * @param a First body
	 * @param b Second body
	 * @param xa x component of anchor point on first body (screen coordinates)
	 * @param ya y component of anchor point on first body (screen coordinates)
	 * @param xb x component of anchor point on second body (screen coordinates)
	 * @param yb y component of anchor point on second body (screen coordinates)
	 * @return Newly created DistanceJoint
	 */
	public DistanceJoint createDistanceJoint(Body a, Body b, float xa, float ya, float xb, float yb) {
		Vec2 va = m_draw.screenToWorld(xa,ya);
		Vec2 vb = m_draw.screenToWorld(xb,yb);
		DistanceJointDef jd = new DistanceJointDef();
		jd.initialize(a, b, va, vb);
		return (DistanceJoint)m_world.createJoint(jd);
	}
	
	/**
	 * Create a revolute (pin) joint between the two bodies
	 * at the given position.
	 * <BR><BR>
	 * Joint limits and motors may be set once the joint is created.
	 * @param a First body
	 * @param b Second body
	 * @param x x coordinate of pin joint location (screen coordinates)
	 * @param y y coordinate of pin joint location (screen coordinates)
	 * @return Newly created RevoluteJoint
	 */
	public RevoluteJoint createRevoluteJoint(Body a, Body b, float x, float y) {
		Vec2 v = m_draw.screenToWorld(x,y);
		return JointUtils.createRevoluteJoint(a, b, v);
	}
	
	/**
	 * Create a prismatic (piston) joint between two bodies
	 * that allows movement in the given direction.
	 * <BR><BR>
	 * dirX and dirY can be given in screen coordinates or
	 * world coordinates, scaling does not matter.
	 * <BR><BR>
	 * Joint limits and motors may be set once the joint is created.
	 * @param a First body
	 * @param b Second body
	 * @param dirX x component of allowed movement direction
	 * @param dirY y component of allowed movement direction
	 * @return Newly created PrismaticJoint
	 */
	public PrismaticJoint createPrismaticJoint(Body a, Body b, float dirX, float dirY) {
		Vec2 dir = new Vec2(dirX, dirY);
		dir.normalize();
		PrismaticJointDef pjd = new PrismaticJointDef();
		pjd.initialize(a, b, (a.getWorldCenter().add(b.getWorldCenter())).mul(0.5f), dir);
		return (PrismaticJoint)m_world.createJoint(pjd);
	}
	
	/**
	 * Create a pulley joint between the 
	 * The pulley joint is connected to two bodies and two fixed ground points.
     * The pulley supports a ratio such that:
     * length1 + ratio * length2 = constant
     * Yes, the force transmitted is scaled by the ratio.
     * <BR><BR>
     * The ground anchors are the points where the "rope" touches the pulley,
     * and the anchors are the points on the bodies where the rope is attached.
     * <BR><BR>
     * Joint limits may be set after the joint is created.
	 * @param a First body
	 * @param b Second body
	 * @param groundAnchorAx x coordinate of (fixed) ground anchor for body a, in screen coordinates
	 * @param groundAnchorAy y coordinate of (fixed) ground anchor for body a, in screen coordinates
	 * @param groundAnchorBx x coordinate of (fixed) ground anchor for body b, in screen coordinates
	 * @param groundAnchorBy y coordinate of (fixed) ground anchor for body b, in screen coordinates
	 * @param anchorAx x coordinate of body anchor for body a, in screen coordinates
	 * @param anchorAy y coordinate of body anchor for body a, in screen coordinates
	 * @param anchorBx x coordinate of body anchor for body b, in screen coordinates
	 * @param anchorBy y coordinate of body anchor for body b, in screen coordinates
	 * @param ratio "Block and tackle" ratio
	 * @return Newly created PulleyJoint
	 */
	public PulleyJoint createPulleyJoint(Body a, Body b, float groundAnchorAx, float groundAnchorAy,
														 float groundAnchorBx, float groundAnchorBy,
														 float anchorAx, float anchorAy,
														 float anchorBx, float anchorBy,
														 float ratio) {
		Vec2 gA = m_draw.screenToWorld(groundAnchorAx,groundAnchorAy);
		Vec2 gB = m_draw.screenToWorld(groundAnchorBx,groundAnchorBy);
		Vec2 aA = m_draw.screenToWorld(anchorAx,anchorAy);
		Vec2 aB = m_draw.screenToWorld(anchorBx,anchorBy);
		PulleyJointDef pjd = new PulleyJointDef();
		pjd.initialize(a, b, gA, gB, aA, aB, ratio);
		return (PulleyJoint)m_world.createJoint(pjd);
	}
	
	/**
	 * Create a gear joint, which binds together two existing
	 * revolute or prismatic joints (any combination will work).
	 * The provided joints must attach a dynamic body to a static body.
	 * <BR><BR>
	 * A gear joint is used to connect two joints together. Either joint
	 * can be a revolute or prismatic joint. You specify a gear ratio
	 * to bind the motions together:
	 * coordinate1 + ratio * coordinate2 = constant
	 * The ratio can be negative or positive. If one joint is a revolute joint
	 * and the other joint is a prismatic joint, then the ratio will have units
	 * of length or units of 1/length.
	 * <BR><em>Warning</em>: The revolute and prismatic joints must be attached to
	 * fixed bodies (which must be body1 on those joints).
	 * @param pj1 First joint (revolute or prismatic)
	 * @param pj2 Second joint (revolute or prismatic)
	 * @param ratio Gear ratio
	 * @return Newly created GearJoint
	 */
	public GearJoint createGearJoint(Joint pj1, Joint pj2, float ratio) {
		if ( !(pj1.getType() == JointType.REVOLUTE_JOINT || pj1.getType() == JointType.PRISMATIC_JOINT) ) {
			throw new IllegalArgumentException("Gear joints can only be created between combinations of revolute and prismatic joints.");
		} else if ( !(pj1.getType() == JointType.REVOLUTE_JOINT || pj1.getType() == JointType.PRISMATIC_JOINT)) {
			throw new IllegalArgumentException("Gear joints can only be created between combinations of revolute and prismatic joints.");
		}
		GearJointDef gjd = new GearJointDef();
		gjd.joint1 = pj1;
		gjd.joint2 = pj2;
		gjd.ratio = ratio;
		return (GearJoint)m_world.createJoint(gjd);
	}
	
	/**
	 * Sets the body def properties based on the current state
	 * of the physics handler.
	 * 
	 * @param bd
	 */
	private void setBodyDefProperties(BodyDef bd) {
		bd.isBullet = m_bullet;
	}
	
	/**
	 * Sets the shape def properties based on the current state
	 * of the physics handler.
	 * 
	 * @param sd Shape def to set
	 */
	private void setShapeDefProperties(ShapeDef sd) {
		sd.density = m_density;
		sd.friction = m_friction;
		sd.restitution = m_restitution;
		sd.isSensor = m_sensor;
	}
	
	/**
	 * Set the density used for newly created shapes.
	 * @param d
	 */
	public void setDensity(float d) {
		m_density = d;
	}
	
	/**
	 * Get the density being used for newly created shapes.
	 * @return
	 */
	public float getDensity() {
		return m_density;
	}
	
	/**
	 * Set the restitution used for newly created shapes.
	 * @param r
	 */
	public void setRestitution(float r) {
		m_restitution = r;
	}
	
	/**
	 * Get the restitution being used for newly created shapes. 
	 * @return
	 */
	public float getRestitution() {
		return m_restitution;
	}
	
	/**
	 * Set the friction used for newly created shapes.
	 * @param f
	 */
	public void setFriction(float f) {
		m_friction = f;
	}
	
	/**
	 * Get the friction being used for newly created shapes.
	 * @return
	 */
	public float getFriction() {
		return m_friction;
	}
	
	/**
	 * Set to true to create new bodies as "bullets,"
	 * which use (slower) continuous collision detection
	 * against other moving bodies.
	 * <BR><BR>
	 * <em>Warning:</em> continuous collision detection between
	 * moving bodies is slow, and should be used sparingly.  All
	 * bodies use continuous collision detection against static
	 * scenery, so for most purposes your bodies should not be
	 * marked as bullets.
	 * @param bullet
	 */
	public void setBullet(boolean bullet) {
		m_bullet = bullet;
	}
	
	/**
	 * Are newly created bodies being created as bullets?
	 */
	public boolean getBullet() {
		return m_bullet;
	}
	
	/**
	 * Set to true to create new shapes as sensors.  Sensors
	 * do not respond to collisions physically, but they
	 * generate contact events.  This can be useful if you
	 * need to check whether a body is in a certain geometrical
	 * area.
	 * @param sensor
	 */
	public void setSensor(boolean sensor) {
		m_sensor = sensor;
	}
	
	/**
	 * Are newly created shapes being created as sensors?
	 */
	public boolean getSensor() {
		return m_sensor;
	}
	
	/**
	 * Destroy this world, unregistering it from the PApplet.
	 * If this is not called, the world will still be active
	 * and simulating, as upon creation it is registered with
	 * the PApplet's draw events.
	 */
	public void destroy() {
		m_parent.unregisterDraw(this);
	}
	
	/**
	 * Get the current physics world.
	 * <BR><BR>
	 * <em>Warning:</em> anything involving a World object directly
	 * is not strictly supported as part of this Processing library.
	 * It <em>is</em> supported as part of JBox2d, however, so there
	 * is quite a bit you can do, and you can always ask for help if
	 * you run into trouble.  Note that all coordinates and vectors
	 * in JBox2d proper are in world coordinates, not screen coordinates,
	 * so you will likely need to use the screenToWorld and worldToScreen
	 * functions to convert back and forth as necessary.
	 * @return The active physics world
	 */
	public World getWorld() {
		return m_world;
	}
	
	/** Remove a body from the world. */
	public void removeBody(Body b) {
		m_world.destroyBody(b);
	}
	
	/** Remove a joint from the world. */
	public void removeJoint(Joint j) {
		m_world.destroyJoint(j);
	}
	
	
	// Screen space to world space conversions
	
	/** Screen space to world space conversion for position. */
	public float screenToWorldX(float x, float y) {
		return m_draw.screenToWorld(x,y).x;
	}
	
	/** Screen space to world space conversion for position. */
	public float screenToWorldY(float x, float y) {
		return m_draw.screenToWorld(x,y).y;
	}
	
	/** Screen space to world space conversion for position. */
	public Vec2 screenToWorld(float x, float y) {
		return m_draw.screenToWorld(x,y);
	}
	
	/** Screen space to world space conversion for position. */
	public Vec2 screenToWorld(Vec2 v) {
		return m_draw.screenToWorld(v);
	}
	
	/** World space to screen space conversion for position. */
	public float worldToScreenX(float x, float y) {
		return m_draw.worldToScreen(x, y).x;
	}
	
	/** World space to screen space conversion for position. */
	public float worldToScreenY(float x, float y) {
		return m_draw.worldToScreen(x, y).y;
	}
	
	/** World space to screen space conversion for position. */
	public Vec2 worldToScreen(float x, float y) {
		return m_draw.worldToScreen(x, y);
	}
	
	/** World space to screen space conversion for length. */
	public float worldToScreen(float length) {
		return m_draw.worldToScreen(length);
	}
	
	/** World space to screen space conversion for position. */
	public Vec2 worldToScreen(Vec2 v) {
		return m_draw.worldToScreen(v);
	}
	
	/** Screen space to world space conversion for length. */
	public float screenToWorld(float length) {
		return m_draw.screenToWorld(length);
	}
	
}
