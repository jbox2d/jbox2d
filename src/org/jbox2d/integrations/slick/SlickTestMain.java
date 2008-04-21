package org.jbox2d.integrations.slick;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.PolygonDef;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BoundaryListener;
import org.jbox2d.dynamics.ContactListener;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.DestructionListener;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.testbed.TestSettings;
import org.newdawn.slick.AppGameContainer;
import org.newdawn.slick.BasicGame;
import org.newdawn.slick.GameContainer;
import org.newdawn.slick.Graphics;
import org.newdawn.slick.SlickException;
import org.newdawn.slick.tests.AntiAliasTest;

/**
 * A bare-bones Slick implementation of the Pyramid demo.
 * Just enough to get to the drawing - no input handling
 * of any sort, and only very basic camera handling.
 * <BR><BR>
 * Should get you on your way, though.
 * 
 * @author ewjordan
 *
 */
public class SlickTestMain extends BasicGame {

	public static void main(String[] args) {
		try {
			AppGameContainer container = new AppGameContainer(new SlickTestMain());
			container.setDisplayMode(600,600,false);
			container.start();
		} catch (SlickException e) {
			e.printStackTrace();
		}
	}

	public SlickTestMain() {
		super("Slick/JBox2d Test");
	}

	@Override
	public void init(GameContainer container) throws SlickException {
		settings = new TestSettings();
		createWorld();
		m_bomb = null;
		m_mouseJoint = null;
		m_points = new ContactPoint[k_maxContactPoints];
		for (int i=0; i<m_points.length; ++i) {
			m_points[i] = new ContactPoint();
		}
		m_destructionListener = new ConcreteDestructionListener();
		m_boundaryListener = new ConcreteBoundaryListener();
		m_contactListener = new ConcreteContactListener();
		((ConcreteDestructionListener)m_destructionListener).test = this;
		((ConcreteBoundaryListener)m_boundaryListener).test = this;
		((ConcreteContactListener)m_contactListener).test = this;
		m_world.setDestructionListener(m_destructionListener);
		m_world.setBoundaryListener(m_boundaryListener);
		m_world.setContactListener(m_contactListener);
		m_debugDraw = new SlickDebugDraw();
		m_world.setDebugDraw(m_debugDraw);
		create();
	}

	@Override
	public void update(GameContainer container, int delta)
			throws SlickException {
	}

	public void render(GameContainer container, Graphics g)
			throws SlickException {
		m_debugDraw.g = g;
		m_debugDraw.container = container;
		step();
	}


	private TestSettings settings;
	private Body m_bomb;
	private MouseJoint m_mouseJoint;
	private ContactPoint[] m_points;
	private ConcreteDestructionListener m_destructionListener;
	private ConcreteBoundaryListener m_boundaryListener;
	private ConcreteContactListener m_contactListener;
	private World m_world;
	private AABB m_worldAABB;
	private int m_pointCount;
	
	public SlickDebugDraw m_debugDraw;
	
	
	/** Override this if you need to create a different world AABB or gravity vector */
	public void createWorld() {
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound = new Vec2(-200.0f, -100.0f);
		m_worldAABB.upperBound = new Vec2(200.0f, 200.0f);
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
	}
	
	public void create() {
		{
			PolygonDef sd = new PolygonDef();
			sd.setAsBox(50.0f, 10.0f);

			BodyDef bd = new BodyDef();
			bd.position.set(0.0f, -10.0f);
			Body ground = m_world.createBody(bd);
			ground.createShape(sd);
		}

		{
			PolygonDef sd = new PolygonDef();
			float a = 0.5f;
			sd.setAsBox(a, a);
			sd.density = 5.0f;
			sd.restitution = 0.0f;
			sd.friction = 0.9f;

			Vec2 x = new Vec2(-10.0f, 0.75f);
			Vec2 y = new Vec2();
			Vec2 deltaX = new Vec2(0.5625f, 2.0f);
			Vec2 deltaY = new Vec2(1.125f, 0.0f);

			for (int i = 0; i < 25; ++i) {
				y.set(x);

				for (int j = i; j < 25; ++j) {
					BodyDef bd = new BodyDef();
					bd.position.set(y);
					Body body = m_world.createBody(bd);
					body.createShape(sd);
					body.setMassFromShapes();

					y.addLocal(deltaY);
				}

				x.addLocal(deltaX);
			}
		}
	}
	
	/**
	 * Take a physics step.  This is the guts of the simulation loop.
	 * When creating your own game, the most important thing to have
	 * in your step method is the m_world.step() call.
	 */
	public void step() {
		float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;
		
		if (settings.pause) {
			if (settings.singleStep) {
				settings.singleStep = false;
			} else {
				timeStep = 0.0f;
			}
		}

		m_debugDraw.setFlags(0);
		if (settings.drawShapes) m_debugDraw.appendFlags(DebugDraw.e_shapeBit);
		if (settings.drawJoints) m_debugDraw.appendFlags(DebugDraw.e_jointBit);
		if (settings.drawCoreShapes) m_debugDraw.appendFlags(DebugDraw.e_coreShapeBit);
		if (settings.drawAABBs) m_debugDraw.appendFlags(DebugDraw.e_aabbBit);
		if (settings.drawOBBs) m_debugDraw.appendFlags(DebugDraw.e_obbBit);
		if (settings.drawPairs) m_debugDraw.appendFlags(DebugDraw.e_pairBit);
		if (settings.drawCOMs) m_debugDraw.appendFlags(DebugDraw.e_centerOfMassBit);

		m_world.setWarmStarting(settings.enableWarmStarting);
		m_world.setPositionCorrection(settings.enablePositionCorrection);
		m_world.setContinuousPhysics(settings.enableTOI);

		m_pointCount = 0;
		
		m_world.step(timeStep, settings.iterationCount);

		//Optional validation of broadphase - asserts if there is an error
		//m_world.m_broadPhase.validate();

		if (m_bomb != null && m_bomb.isFrozen()) {
			m_world.destroyBody(m_bomb);
			m_bomb = null;
		}

		if (m_mouseJoint != null) {
			Body body = m_mouseJoint.m_body2;
			Vec2 p1 = body.getWorldPoint(m_mouseJoint.m_localAnchor);
			Vec2 p2 = m_mouseJoint.m_target;

			m_debugDraw.drawSegment(p1, p2, new Color3f(255.0f,255.0f,255.0f));
		}

		if (settings.drawContactPoints) {
			float k_forceScale = 0.01f;
			float k_axisScale = 0.3f;

			for (int i = 0; i < m_pointCount; ++i) {
				ContactPoint point = m_points[i];
				
				if (point.state == 0) {
					// Add
					//System.out.println("Add");
					m_debugDraw.drawPoint(point.position, 0.3f, new Color3f(255.0f, 150.0f, 150.0f));
				} else if (point.state == 1) {
					// Persist
					//System.out.println("Persist");
					m_debugDraw.drawPoint(point.position, 0.1f, new Color3f(255.0f, 0.0f, 0.0f));
				} else {
					// Remove
					//System.out.println("Remove");
					m_debugDraw.drawPoint(point.position, 0.5f, new Color3f(0.0f, 155.0f, 155.0f));
				}

				if (settings.drawContactNormals) {
					Vec2 p1 = point.position;
					Vec2 p2 = new Vec2( p1.x + k_axisScale * point.normal.x,
										p1.y + k_axisScale * point.normal.y);
					m_debugDraw.drawSegment(p1, p2, new Color3f(0.4f*255f, 0.9f*255f, 0.4f*255f));
				} else if (settings.drawContactForces) {
					Vec2 p1 = point.position;
					Vec2 p2 = new Vec2( p1.x + k_forceScale * point.normalForce * point.normal.x,
										p1.y + k_forceScale * point.normalForce * point.normal.y);
					m_debugDraw.drawSegment(p1, p2, new Color3f(0.9f*255f, 0.9f*255f, 0.3f*255f));
				}

				if (settings.drawFrictionForces) {
					Vec2 tangent = Vec2.cross(point.normal, 1.0f);
					Vec2 p1 = point.position;
					Vec2 p2 = new Vec2( p1.x + k_forceScale * point.tangentForce * tangent.x,
										p1.y + k_forceScale * point.tangentForce * tangent.y);
					m_debugDraw.drawSegment(p1, p2, new Color3f(0.9f*255f, 0.9f*255f, 0.3f*255f));
				}
			}
		}
		
	}
	
	
	public void boundaryViolated(Body b) {
		;
	}
	
	public void jointDestroyed(Joint j) {
		;
	}
	

	/* ==== Concrete listener classes below ====
	 * Concrete tests may override these classes, just
	 * remember to call m_world.setListener(newListener)
	 */       
	
	/**
	 * This is called when a joint in the world is implicitly destroyed
	 *	because an attached body is destroyed. This gives us a chance to
	 *	nullify the mouse joint.
	 */
    class ConcreteDestructionListener implements DestructionListener {
    	public void sayGoodbye(Shape shape) {; }
    	public void sayGoodbye(Joint joint) {
    		if (test.m_mouseJoint == joint) {
    			test.m_mouseJoint = null;
    		} else {
    			test.jointDestroyed(joint);
    		}
    	}

    	public SlickTestMain test;
    }

    /**
     * Calls boundaryViolated(Body) on violation.
     *
     */
    class ConcreteBoundaryListener implements BoundaryListener {
    	public void violation(Body body) {
    		if (test.m_bomb != body) {
    			test.boundaryViolated(body);
    		}
    	}

    	public SlickTestMain test;
    }

    /** Max number of contact points to store */
    static final int k_maxContactPoints = 2048; 

    /**
     * Stores contact points away for inspection, as with CCD
     * contacts may be gone by the end of a step.
     * 
     */
    class ConcreteContactListener implements ContactListener {
    	public void add(org.jbox2d.dynamics.contacts.ContactPoint point) {
    		if (test.m_pointCount == k_maxContactPoints) {
    			return;
    		}

    		ContactPoint cp = test.m_points[test.m_pointCount];
    		cp.shape1 = point.shape1;
    		cp.shape2 = point.shape2;
    		cp.position = point.position.clone();
    		cp.normal = point.normal.clone();
    		cp.normalForce = point.normalForce;
    		cp.tangentForce = point.tangentForce;
    		cp.state = 0;

    		++test.m_pointCount;	
    	}
    	public void persist(org.jbox2d.dynamics.contacts.ContactPoint point) {
    		if (test.m_pointCount == k_maxContactPoints) {
    			return;
    		}

    		ContactPoint cp = test.m_points[test.m_pointCount];
    		cp.shape1 = point.shape1;
    		cp.shape2 = point.shape2;
    		cp.position = point.position.clone();
    		cp.normal = point.normal.clone();
    		cp.normalForce = point.normalForce;
    		cp.tangentForce = point.tangentForce;
    		cp.state = 1;

    		++test.m_pointCount;
    	}
    	public void remove(org.jbox2d.dynamics.contacts.ContactPoint point) {
    		if (test.m_pointCount == k_maxContactPoints) {
    			return;
    		}

    		ContactPoint cp = test.m_points[test.m_pointCount];
    		cp.shape1 = point.shape1;
    		cp.shape2 = point.shape2;
    		cp.position = point.position.clone();
    		cp.normal = point.normal.clone();
    		cp.normalForce = point.normalForce;
    		cp.tangentForce = point.tangentForce;
    		cp.state = 2;

    		++test.m_pointCount;
    	}

    	public SlickTestMain test;
    }

    /**
     * Holder for storing contact information.
     * Not the same as org.jbox2d.dynamics.contacts.ContactPoint
     */
    class ContactPoint {
    	public Shape shape1;
    	public Shape shape2;
    	public Vec2 normal;
    	public Vec2 position;
    	public float normalForce;
    	public float tangentForce;
    	public int state; // 0-add, 1-persist, 2-remove
    }

}
