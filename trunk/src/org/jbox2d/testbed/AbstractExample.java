package org.jbox2d.testbed;

import java.util.ArrayList;

import javax.vecmath.Color3f;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.CircleDef;
import org.jbox2d.collision.Shape;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BoundaryListener;
import org.jbox2d.dynamics.ContactListener;
import org.jbox2d.dynamics.DebugDraw;
import org.jbox2d.dynamics.DestructionListener;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.ContactPoint;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.MouseJointDef;

public abstract class AbstractExample {
	public TestbedMain parent;    //controller applet/app
	public DebugDraw m_debugDraw;
	public Body followedBody = null; //camera follows motion of this body
    public boolean[] keyDown = new boolean[255];
    public boolean[] newKeyDown = new boolean[255];

    public Vec2 mouseScreen = new Vec2(); // screen coordinates of mouse
    public Vec2 mouseWorld = new Vec2(); // world coordinates of mouse
    public Vec2 pmouseScreen = new Vec2();
    public boolean pmousePressed; // was mouse pressed last frame?
    protected boolean needsReset = true;//True if we should reset the demo on the next loop
    protected Vec2 bombSpawnPoint;
    protected boolean bombSpawning;
    protected int m_textLine;
    protected int m_pointCount;
    protected ContactPoint[] m_points;
    protected World m_world;
    protected Body m_bomb;
    protected MouseJoint m_mouseJoint;
    protected TestSettings settings;
	protected AABB m_worldAABB;
	
	protected DestructionListener m_destructionListener;
	protected BoundaryListener m_boundaryListener;
	protected ContactListener m_contactListener;
	
    //protected ArrayList<BoundImage> boundImages = new ArrayList<BoundImage>();
	
    /**
     * @return Title of example
     */
	abstract public String getName();
    
	/**
	 * Create the world geometry for each test.
	 * Any custom initialization for a test can usually go here.
	 * 
	 * Called immediately after initialize(), which handles
	 * generic test initialization.
	 */
	abstract public void create();
	
	public AbstractExample(TestbedMain _parent) {
		parent = _parent;
		m_debugDraw = parent.g;
		needsReset = true;
	}
	
	public void initialize() {
		needsReset = false;
		m_textLine = 30;
		for (int i=0; i<255; ++i) {
			keyDown[i] = false;
			newKeyDown[i] = false;
		}
		settings = new TestSettings();
		mouseScreen = new Vec2(parent.mouseX, parent.mouseY);
		mouseWorld = new Vec2();
		pmouseScreen = new Vec2(mouseScreen.x,mouseScreen.y);
		pmousePressed = false;
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound = new Vec2(-200.0f, -100.0f);
		m_worldAABB.upperBound = new Vec2(200.0f, 200.0f);
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
		
		m_bomb = null;
		m_mouseJoint = null;
		bombSpawnPoint = null;
		bombSpawning = false;
		
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
		m_world.setListener(m_destructionListener);
		m_world.setListener(m_boundaryListener);
		m_world.setListener(m_contactListener);
		m_world.setDebugDraw(parent.g);
		create();
	}
	
	public void step() {
		m_textLine = 15;
		mouseWorld.set(m_debugDraw.screenToWorld(mouseScreen));
		//System.out.println(mouseWorld);
		float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;

		if (settings.pause) {
			if (settings.singleStep) {
				settings.singleStep = false;
			} else {
				timeStep = 0.0f;
			}

			//m_debugDraw.drawString(5, m_textLine, "****PAUSED****", new Color3f(1.0f,1.0f,1.0f));
			m_textLine += 15;
		}

		m_debugDraw.setFlags(0);
		if (settings.drawShapes) m_debugDraw.appendFlags(DebugDraw.e_shapeBit);
		if (settings.drawJoints) m_debugDraw.appendFlags(DebugDraw.e_jointBit);
		if (settings.drawCoreShapes) m_debugDraw.appendFlags(DebugDraw.e_coreShapeBit);
		if (settings.drawAABBs) m_debugDraw.appendFlags(DebugDraw.e_aabbBit);
		if (settings.drawOBBs) m_debugDraw.appendFlags(DebugDraw.e_obbBit);
		if (settings.drawPairs) m_debugDraw.appendFlags(DebugDraw.e_pairBit);
		if (settings.drawCOMs) m_debugDraw.appendFlags(DebugDraw.e_centerOfMassBit);

		World.ENABLE_WARM_STARTING = settings.enableWarmStarting;
		World.ENABLE_POSITION_CORRECTION = settings.enablePositionCorrection;
		World.ENABLE_TOI = settings.enableTOI;

		m_pointCount = 0;
		
		m_world.step(timeStep, settings.iterationCount);

		m_world.m_broadPhase.validate();

		if (m_bomb != null && m_bomb.isFrozen()) {
			m_world.destroyBody(m_bomb);
			m_bomb = null;
		}

		if (settings.drawStats) {
			Color3f white = new Color3f(1.0f, 1.0f, 1.0f);
			m_debugDraw.drawString(5, m_textLine, "proxies(max) = "+m_world.m_broadPhase.m_proxyCount+
					"("+Settings.maxProxies+"), pairs(max) = "+m_world.m_broadPhase.m_pairManager.m_pairCount+
					"("+Settings.maxPairs+")", white);
			m_textLine += 15;

			m_debugDraw.drawString(5, m_textLine, "bodies/contacts/joints = "+
				m_world.m_bodyCount+"/"+m_world.m_contactCount+"/"+m_world.m_jointCount, white);
			m_textLine += 15;

			m_debugDraw.drawString(5, m_textLine, "position iterations = "+m_world.m_positionIterationCount, white);
			m_textLine += 15;

			long memTot = Runtime.getRuntime().totalMemory();
			long memFree =  Runtime.getRuntime().freeMemory();
			m_debugDraw.drawString(5, m_textLine, "total memory: "+memTot, white);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "free memory: "+memFree, white);
			m_textLine += 15;
		}

		if (m_mouseJoint != null) {
			Body body = m_mouseJoint.m_body2;
			Vec2 p1 = body.getWorldPoint(m_mouseJoint.m_localAnchor);
			Vec2 p2 = m_mouseJoint.m_target;

			m_debugDraw.drawSegment(p1, p2, new Color3f(255.0f,255.0f,255.0f));
		}
		
		if (bombSpawning) {
			m_debugDraw.drawSolidCircle(bombSpawnPoint, 0.3f, new Vec2(1.0f,0.0f),new Color3f(255f*0.5f,255f*0.5f,255f*0.5f));
			m_debugDraw.drawSegment(bombSpawnPoint, mouseWorld, new Color3f(55f*0.5f,55f*0.5f,255f*0.5f));
		}

		if (settings.drawContactPoints) {
			float k_forceScale = 0.01f;
			float k_axisScale = 0.3f;

			for (int i = 0; i < m_pointCount; ++i) {
				ContactPoint point = m_points[i];
				
				if (point.state == 0) {
					// Add
					//System.out.println("Add");
					m_debugDraw.drawPoint(point.position, 10.0f, new Color3f(255.0f, 150.0f, 150.0f));
				} else if (point.state == 1) {
					// Persist
					//System.out.println("Persist");
					m_debugDraw.drawPoint(point.position, 5.0f, new Color3f(255.0f, 0.0f, 0.0f));
				} else {
					// Remove
					//System.out.println("Remove");
					m_debugDraw.drawPoint(point.position, 10.0f, new Color3f(155f, 0.0f, 0.0f));
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
		
		for (int i = 0; i < newKeyDown.length; i++) {
            newKeyDown[i] = false;
        }
		pmouseScreen.set(mouseScreen);
	}

	//Space launches a bomb from a random default position
    public void launchBomb() {
    	Vec2 pos = new Vec2(parent.random(-15.0f, 15.0f), 30.0f);
    	Vec2 vel = pos.mul(-5.0f);
    	launchBomb(pos, vel);
    }
    	
    public void launchBomb(Vec2 position, Vec2 velocity) {
    	if (m_bomb != null) {
    		m_world.destroyBody(m_bomb);
    		m_bomb = null;
    	}

    	BodyDef bd = new BodyDef();
    	bd.allowSleep = true;
    	bd.position = position.clone();
    	bd.isBullet = true;
    	m_bomb = m_world.createDynamicBody(bd);
    	m_bomb.setLinearVelocity(velocity);

    	CircleDef sd = new CircleDef();
    	sd.radius = 0.3f;
    	sd.density = 20.0f;
    	sd.restitution = 0.1f;
    	m_bomb.createShape(sd);
    	
    	m_bomb.setMassFromShapes();
    }
    
    //Shift+drag "slingshots" a bomb from any point using these functions
    public void spawnBomb(Vec2 worldPt) {
    	bombSpawnPoint = worldPt.clone();
    	bombSpawning = true;
    }
    
    public void completeBombSpawn() {
    	final float multiplier = 30.0f;
    	Vec2 mouseW = m_debugDraw.screenToWorld(mouseScreen);
    	Vec2 vel = bombSpawnPoint.sub(mouseW);
    	vel.mulLocal(multiplier);
    	launchBomb(bombSpawnPoint,vel);
    	bombSpawning = false;
    }
    
    public void keyPressed(int key) {
        if (key >= 0 && key < 255) {
            if (!keyDown[key])
                newKeyDown[key] = true;
            keyDown[key] = true;
        }
    }

    public void keyReleased(int key) {
        if (key >= 0 && key < 255) {
            keyDown[key] = false;
        }
    }
    
    public void mouseDown(Vec2 p) {
    	
    	if (parent.shiftKey) {
    		spawnBomb(m_debugDraw.screenToWorld(p));
    		return;
    	}
    	
    	p = m_debugDraw.screenToWorld(p);
    	
    	assert m_mouseJoint == null;

        // Make a small box.

        Vec2 d = new Vec2(0.001f, 0.001f);
        AABB aabb = new AABB(p.sub(d), p.add(d));

        // Query the world for overlapping shapes.
        int k_maxCount = 10;
        Shape shapes[] = m_world.query(aabb, k_maxCount);
        
        Body body = null;
        for (int j = 0; j < shapes.length; j++) {
            Body shapeBody = shapes[j].getBody();
            if (shapeBody.isStatic() == false) {
                boolean inside = shapes[j].testPoint(shapeBody.getXForm(),p);
                if (inside) {
                    body = shapes[j].m_body;
                    break;
                }
            }
        }

        if (body != null) {
            MouseJointDef md = new MouseJointDef();
            md.body1 = m_world.m_groundBody;
            md.body2 = body;
            md.target.set(p);
            md.maxForce = 1000.0f * body.m_mass;
            m_mouseJoint = (MouseJoint) m_world.createJoint(md);
            body.wakeUp();
        }
    }

    public void mouseUp() {
        if (m_mouseJoint != null) {
            m_world.destroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }
        if (bombSpawning) {
        	completeBombSpawn();
        }
    }

    public void mouseMove(Vec2 p) {
    	mouseScreen.set(p);
        if (m_mouseJoint != null) {
            m_mouseJoint.setTarget(m_debugDraw.screenToWorld(p));
        }
    }
    
	
    /** Stub method for concrete examples to override if desired.
     *  Called when a joint is implicitly destroyed due to body
     *  destruction.
     *  
     *  @param joint The implicitly destroyed joint
     */
	public void jointDestroyed(Joint joint) {
		
	}

	/** Stub method for concrete examples to override if desired.
	 *  Called when a body leaves the world boundary.
	 *  
	 * @param body The body that went out of bounds
	 */
	public void boundaryViolated(Body body) {
		
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

    	public AbstractExample test;
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

    	public AbstractExample test;
    }

    static final int k_maxContactPoints = 2048; //max # contacts to store

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

    	public AbstractExample test;
    }

    /**
     * Holder for contact information.
     * Different from org.jbox2d.dynamics.contacts.ContactPoint
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
