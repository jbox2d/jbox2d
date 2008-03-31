/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.testbed;

import java.util.ArrayList;
import processing.core.PImage;

import org.jbox2d.common.Color3f;
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
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.MouseJointDef;

public abstract class AbstractExample {
	/** The controller that the AbstractExample runs in */
	public TestbedMain parent;
	/** Used for drawing */
	public DebugDraw m_debugDraw;
	//public Body followedBody = null; //camera follows motion of this body
	/** Array of key states, by char value.  Does not include arrows or modifier keys. */
    public boolean[] keyDown = new boolean[255];
    /** Same as keyDown, but true only if the key was newly pressed this frame. */
    public boolean[] newKeyDown = new boolean[255];
    
    /** General instructions that apply to all tests. */
    static public String instructionString = "Press left/right to change test\n" +
    										 "Use the mouse to drag objects\n" +
    										 "Shift+drag to slingshot bomb\n" +
    										 "Press 'o' to toggle options panel\n";
    										
    /** Screen coordinates of mouse */
    public Vec2 mouseScreen = new Vec2();
    /** World coordinates of mouse */
    public Vec2 mouseWorld = new Vec2();
    /** Screen coordinates of mouse on last frame */
    public Vec2 pmouseScreen = new Vec2();
    /** Was the mouse pressed last frame?  True if either right or left button was down. */
    public boolean pmousePressed;
    /** True if we should reset the demo for the next frame. */
    protected boolean needsReset = true;
    /** The point at which we will place a bomb when completeBombSpawn() is called. */
    protected Vec2 bombSpawnPoint;
    /** True if a bomb has started spawning but has not been created yet. */
    protected boolean bombSpawning;
    /** Y-pixel value that marks bottom of text to be drawn. */
    protected int m_textLine;
    /** Number of active points in m_points array. */
    protected int m_pointCount;
    /** Array of contact points - use m_pointCount to get number of active elements.  */
    protected ContactPoint[] m_points;
    /** The world object this example uses. */
    protected World m_world;
    /** The bomb body.  May be null if no bomb is active. */
    protected Body m_bomb;
    /** Mouse joint.  May be null if mouse is not attached to anything. */
    protected MouseJoint m_mouseJoint;
    /** Settings for this example.  This is stored and reloaded when the example restarts or we come back from another example. */ 
    protected TestSettings settings;
    /** The bounding box for the world.  If the defaults do not work for you, overload createWorld() and set the AABB appropriately there. */
	protected AABB m_worldAABB;
	/** The exponentially smoothed amount of free memory available to the JVM. */ 
	public float memFree = 0;
	
	/** Listener for body and joint destructions. */
	protected DestructionListener m_destructionListener;
	/** Listener for world AABB violations. */
	protected BoundaryListener m_boundaryListener;
	/** Listener for contact events. */
	protected ContactListener m_contactListener;
	
	public static Color3f white = new Color3f(255.0f,255.0f,255.0f);
	public static Color3f black = new Color3f(0.0f*255.0f,0.0f*255.0f,0.0f*255.0f);
	public static Color3f gray = new Color3f(0.5f*255.0f,0.5f*255.0f,0.5f*255.0f);
	public static Color3f red = new Color3f(255.0f,0.0f,0.0f);
	public static Color3f green = new Color3f(0.0f,255.0f,0.0f);
	public static Color3f blue= new Color3f(0.0f,0.0f,255.0f);
	
	/** Saved camera variable so that camera stays put between reloads of example. */
	public float cachedCamX, cachedCamY, cachedCamScale;
	/** Have the cachedCam* variables been set for this example? */
	public boolean hasCachedCamera = false;
	
	/** Height of font used to draw text. */
	static public int textLineHeight = 12;
	
	/** List of images bound to bodies. */
	protected ArrayList<BoundImage> boundImages = new ArrayList<BoundImage>();
	
	/**
	 * Prints default instructions + specific example instructions.
	 * To add instructions to your example, override getExampleInstructions()
	 */
	public void printInstructions() {
		String fullString = instructionString + getExampleInstructions();
		String[] instructionLines = fullString.split("\n");
		int currentLine = parent.height - instructionLines.length*textLineHeight;
		for (int i=0; i<instructionLines.length; ++i) {
			m_debugDraw.drawString(5, currentLine, instructionLines[i], white);
			currentLine += textLineHeight;
		}
	}
	
	/** 
	 * Returns a string containing example instructions.
	 * Overload within an example to provide special instructions
	 * or information. 
	 * @return A string containing example instructions
	 */
	public String getExampleInstructions() {
		return "";
	}
	
    /**
     * @return Title of example.
     */
	abstract public String getName();
    
	/**
	 * Create the world geometry for each test.
	 * Any custom initialization for a test should go here.
	 * 
	 * Called immediately after initialize(), which handles
	 * generic test initialization and should usually not be overloaded.
	 */
	abstract public void create();
	
	/**
	 * Instantiate the test.
	 * @param _parent The controller that this test is run from.
	 */
	public AbstractExample(TestbedMain _parent) {
		parent = _parent;
		m_debugDraw = parent.g;
		needsReset = true;
	}
	
	/** Overload this if you need to create a different world AABB or gravity vector */
	public void createWorld() {
		m_worldAABB = new AABB();
		m_worldAABB.lowerBound = new Vec2(-200.0f, -100.0f);
		m_worldAABB.upperBound = new Vec2(200.0f, 200.0f);
		Vec2 gravity = new Vec2(0.0f, -10.0f);
		boolean doSleep = true;
		m_world = new World(m_worldAABB, gravity, doSleep);
	}
	
	/**
	 * Should not usually be overloaded.
	 * Performs initialization tasks common to most examples:
	 * <BR>
	 * <UL>
	 * <LI>Resets input state
	 * <LI>Initializes member variables
	 * <LI>Sets test settings
	 * <LI>Creates world, gravity and AABB by calling createWorld() - overload that if you are unhappy with defaults
	 * <LI>Sets up and attaches listeners
	 * <LI>Sets up drawing
	 * <LI>Sets camera to default or saved state
	 * <LI>Calls create() to set up test
	 * </UL>
	 */
	public void initialize() {
		needsReset = false;
		m_textLine = 15;
		for (int i=0; i<255; ++i) {
			keyDown[i] = false;
			newKeyDown[i] = false;
		}
		settings = new TestSettings();
		mouseScreen = new Vec2(parent.mouseX, parent.mouseY);
		mouseWorld = new Vec2();
		pmouseScreen = new Vec2(mouseScreen.x,mouseScreen.y);
		pmousePressed = false;
		
		createWorld();
		
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
		if (hasCachedCamera) {
			m_debugDraw.setCamera(cachedCamX,cachedCamY,cachedCamScale);
		} else {
			m_debugDraw.setCamera(0.0f, 10.0f, 10.0f);
			hasCachedCamera = true;
			cachedCamX = 0.0f;
			cachedCamY = 10.0f;
			cachedCamScale = 10.0f;
		}
		boundImages.clear();
		create();
	}
	
	/**
	 * Take a physics step.  This is the guts of the simulation loop.
	 * When creating your own game, the most important thing to have
	 * in your step method is the m_world.step() call.
	 */
	public void step() {
		preStep();
		mouseWorld.set(m_debugDraw.screenToWorld(mouseScreen));
		
		float timeStep = settings.hz > 0.0f ? 1.0f / settings.hz : 0.0f;
		
		if (settings.pause) {
			if (settings.singleStep) {
				settings.singleStep = false;
			} else {
				timeStep = 0.0f;
			}

			m_debugDraw.drawString(5, m_textLine, "****PAUSED - press '+' to take a single step, 'p' to unpause****", white);
			m_textLine += textLineHeight;
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

		//Optional validation of broadphase - asserts if there is an error
		//m_world.m_broadPhase.validate();

		if (m_bomb != null && m_bomb.isFrozen()) {
			m_world.destroyBody(m_bomb);
			m_bomb = null;
		}

		if (settings.drawStats) {
			m_debugDraw.drawString(5, m_textLine, "proxies(max) = "+m_world.m_broadPhase.m_proxyCount+
					"("+Settings.maxProxies+"), pairs(max) = "+m_world.m_broadPhase.m_pairManager.m_pairCount+
					"("+Settings.maxPairs+")", white);
			m_textLine += textLineHeight;

			m_debugDraw.drawString(5, m_textLine, "bodies/contacts/joints = "+
				m_world.m_bodyCount+"/"+m_world.m_contactCount+"/"+m_world.m_jointCount, white);
			m_textLine += textLineHeight;

			m_debugDraw.drawString(5, m_textLine, "position iterations = "+m_world.m_positionIterationCount, white);
			m_textLine += textLineHeight;

			long memTot = Runtime.getRuntime().totalMemory();
			memFree = (memFree * .9f + .1f * Runtime.getRuntime().freeMemory());
			m_debugDraw.drawString(5, m_textLine, "total memory: "+memTot, white);
			m_textLine += textLineHeight;
			m_debugDraw.drawString(5, m_textLine, "Average free memory: "+(long)memFree, white);
			m_textLine += textLineHeight;
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
		
		for (BoundImage b:boundImages) {
            b.draw();
        }
		
		printInstructions();
		
		pmouseScreen.set(mouseScreen);
		postStep();

		//Should reset newKeyDown after postStep in case it needs to be used there
		for (int i=0; i<newKeyDown.length; ++i) {
			newKeyDown[i] = false;
		}
	}
	
	/** Stub for overloading in examples - called before physics step. */
	public void preStep() {
		
	}
	
	/** Stub for overloading in examples - called after physics step. */
	public void postStep() {
		
	}

	/** Space launches a bomb from a random default position. */
    public void launchBomb() {
    	Vec2 pos = new Vec2(parent.random(-15.0f, 15.0f), 30.0f);
    	Vec2 vel = pos.mul(-5.0f);
    	launchBomb(pos, vel);
    }
    	
    /** 
     * Launch bomb from a specific position with a given velocity.
     * @param position Position to launch bomb from.
     * @param velocity Velocity to launch bomb with.
     */
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

    	Vec2 minV = position.sub(new Vec2(0.3f,0.3f));
    	Vec2 maxV = position.add(new Vec2(0.3f,0.3f));
    	AABB aabb = new AABB(minV, maxV);
    	boolean inRange = m_world.m_broadPhase.inRange(aabb);

    	if (inRange) {
    		m_bomb.createShape(sd);
    		m_bomb.setMassFromShapes();
    	} else {
    		System.out.println("Bomb not created - out of world AABB");
    	}
    }
    
    //Shift+drag "slingshots" a bomb from any point using these functions
    /**
     * Begins spawning a bomb, spawn finishes and bomb is created upon calling completeBombSpawn().
     * When a bomb is spawning, it is not an active body but its position is stored so it may be
     * drawn.
     */
    public void spawnBomb(Vec2 worldPt) {
    	bombSpawnPoint = worldPt.clone();
    	bombSpawning = true;
    }
    
    /**
     * Creates and launches a bomb using the current bomb and mouse locations to "slingshot" it.
     */
    public void completeBombSpawn() {
    	if (!bombSpawning) return;
    	final float multiplier = 30.0f;
    	Vec2 mouseW = m_debugDraw.screenToWorld(mouseScreen);
    	Vec2 vel = bombSpawnPoint.sub(mouseW);
    	vel.mulLocal(multiplier);
    	launchBomb(bombSpawnPoint,vel);
    	bombSpawning = false;
    }

    
    /**
     * Draws an image on a body.
     * 
     * First image is centered on body center, then
     * localScale is applied, then localOffset, and
     * lastly localRotation (all rel. to body center).
     * 
     * Thus localOffset should be specified in body
     * units to the scaled image.  For instance, if
     * you want a MxN image to have its corner
     * at body center and be scaled by S, use a localOffset
     * of (M*S/2, N*S/2) and a localScale of S.
     * 
     */
    public void bindImage(PImage p, Vec2 localOffset, float localRotation, float localScale, Body b) {
        boundImages.add(new BoundImage(p, localOffset, localRotation, localScale, b));
    }
    
    
    /**
     * Set keyDown and newKeyDown arrays when we get a keypress.
     * @param key The key pressed.
     */
    public void keyPressed(int key) {
        if (key >= 0 && key < 255) {
        	//System.out.println(key + " "+keyDown[key]);
            if (!keyDown[key]) newKeyDown[key] = true;
            keyDown[key] = true;
        }
    }
    
    /**
     * Set keyDown array when we get a key release.
     * @param key The key released.
     */
    public void keyReleased(int key) {
        if (key >= 0 && key < 255) {
            keyDown[key] = false;
        }
    }
    
    /**
     * Handle mouseDown events.
     * @param p The screen location that the mouse is down at.
     */
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

    /**
     * Handle mouseUp events.
     */
    public void mouseUp() {
        if (m_mouseJoint != null) {
            m_world.destroyJoint(m_mouseJoint);
            m_mouseJoint = null;
        }
        if (bombSpawning) {
        	completeBombSpawn();
        }
    }

    /**
     * Handle mouseMove events (TestbedMain also sends mouseDragged events here)
     * @param p The new mouse location (screen coordinates)
     */
    public void mouseMove(Vec2 p) {
    	mouseScreen.set(p);
        if (m_mouseJoint != null) {
            m_mouseJoint.setTarget(m_debugDraw.screenToWorld(p));
        }
    }
    
    /**
     * Sets the camera target and scale.
     * 
     * @param x World x coordinate of camera focus
     * @param y World y coordinate of camera focus
     * @param scale Size in screen units (usually pixels) of one world unit (meter)
     */
    public void setCamera(float x, float y, float scale) {
    	m_debugDraw.setCamera(x, y, scale);
    	hasCachedCamera = true;
    	cachedCamX = x;
    	cachedCamY = y;
    	cachedCamScale = scale;
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

    	public AbstractExample test;
    }

    /**
     * Holder for storing contact information.
     * Not the same as org.jbox2d.dynamics.contacts.ContactPoint (TODO: fix name clash)
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
    
    /**
     * Holder for images to be drawn on bodies.
     * You should not need to create BoundImages yourself -
     * instead, use bindImage(), which will properly
     * add the BoundImage to the ArrayList of BoundImages
     * to draw.
     * <BR><BR>
     * In a realistic application, you would also want to
     * decouple the BoundImages from bodies upon body
     * destruction; here we don't do that because this is
     * just a simple example of how to do the drawing based
     * on the body transform.  You also might want to allow
     * height/width scaling not drawn directly
     * from the image in case the image needs stretching.
     * <BR><BR>
     * Necessarily tied to ProcessingDebugDraw and
     * Processing's PImage class because of image
     * format and handling.
     * 
     */
    class BoundImage{
        private PImage image;
        private float halfImageWidth;
        private float halfImageHeight;
        private Body body;
        private Vec2 localOffset;
        private float localRotation;
        private float localScale;
        private ProcessingDebugDraw p;
        
        public BoundImage(PImage _image, Vec2 _localOffset, float _localRotation, float _localScale, Body _body) {
            image = _image;
            localOffset = _localOffset.clone();
            localRotation = _localRotation;
            localScale = _localScale;
            body = _body;
            halfImageWidth = image.width / 2f;
            halfImageHeight = image.height / 2f;
        	p = (ProcessingDebugDraw)m_debugDraw;
        }
        
        public void draw() {
        	p.drawImage(image, body.getPosition(), body.getAngle()+localRotation, localScale, localOffset, halfImageWidth, halfImageHeight);
        }
    }
    
}
