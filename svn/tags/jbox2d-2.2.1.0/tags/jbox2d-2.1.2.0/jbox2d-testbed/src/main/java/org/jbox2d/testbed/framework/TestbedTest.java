/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 2:21:03 PM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.util.LinkedList;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.DestructionListener;
import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Collision.PointState;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.WorldManifold;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.MouseJoint;
import org.jbox2d.dynamics.joints.MouseJointDef;

/**
 * @author Daniel Murphy
 */
public abstract class TestbedTest implements ContactListener{
	public static final int MAX_CONTACT_POINTS = 2048;

	// keep these static so we don't have to recreate them every time
	public final static ContactPoint[] points = new ContactPoint[MAX_CONTACT_POINTS];
	static {
		for(int i=0; i<MAX_CONTACT_POINTS; i++){
			points[i] = new ContactPoint();
		}
	}
	
	public Body m_groundBody;
	protected int m_pointCount;
	private DestructionListener destructionListener;
	public DebugDraw m_debugDraw;
	public World m_world;
	protected Body m_bomb;
	public MouseJoint m_mouseJoint;
	private final Vec2 bombSpawnPoint = new Vec2();
	private boolean bombSpawning = false;
	public final Vec2 m_mouseWorld = new Vec2();
	protected int m_stepCount;
	
	private final LinkedList<QueueItem> inputQueue;
	
	private String title = null;
	protected int m_textLine;
	private final LinkedList<String> textList = new LinkedList<String>();
	
	public float cachedCameraScale;
	public float cachedCameraX;
	public float cachedCameraY;
	
	public boolean hasCachedCamera = false;
	
	private TestPanel panel;
	
	public TestbedTest(){
		inputQueue = new LinkedList<QueueItem>();
	}
	
	public void setPanel(TestPanel argPanel){
		panel = argPanel;
	}
	
	public void init(DebugDraw argDebugDraw){
		m_debugDraw = argDebugDraw;
		destructionListener = new DestructionListener() {
			
			public void sayGoodbye(Fixture fixture) {
			}
			
			public void sayGoodbye(Joint joint) {
				if(m_mouseJoint == joint){
					m_mouseJoint = null;
				}else{
					jointDestroyed(joint);
				}
			}
		};
		
		Vec2 gravity = new Vec2(0, -10f);
		m_world = new World(gravity, true);
		m_bomb = null;
		m_textLine = 30;
		m_mouseJoint = null;
		m_pointCount = 0;
		
		m_world.setDestructionListener(destructionListener);
		m_world.setContactListener(this);
		m_world.setDebugDraw(m_debugDraw);
		
		bombSpawning = false;
		
		m_stepCount = 0;
		
//		Contact.activeContacts = 0;
		
		BodyDef bodyDef = new BodyDef();
		m_groundBody = m_world.createBody(bodyDef);
		
		if(hasCachedCamera){
			setCamera(cachedCameraX, cachedCameraY, cachedCameraScale);
		}else{
			setCamera(0, 10, 10);			
		}
		setTitle(getTestName());
		
		initTest();
	}
	
	public void setCamera(float x, float y, float scale){
		m_debugDraw.setCamera(x, y, scale);
		hasCachedCamera = true;
		cachedCameraScale = scale;
		cachedCameraX = x;
		cachedCameraY = y;
	}
	
	public abstract void initTest();
	
	public abstract String getTestName();
	
	public void update(TestbedSettings settings){
		m_textLine = 15;
		// keys!
		if(TestPanel.keys['r']){
			TestPanel.keys['r'] = false;
			init(m_debugDraw);
		}
		
		if(title != null){
			m_debugDraw.drawString(panel.getWidth()/2, 15, title, Color3f.WHITE);
		}
		
		// process our input
		if(!inputQueue.isEmpty()){
			synchronized (inputQueue) {
				while(!inputQueue.isEmpty()){
					QueueItem i = inputQueue.pop();
					switch(i.type){
						case KeyPressed:
							keyPressed(i.c, i.code);
							break;
						case KeyReleased:
							keyReleased(i.c, i.code);
							break;
						case MouseDown:
							mouseDown(i.p);
							break;
						case MouseMove:
							mouseMove(i.p);
							break;
						case MouseUp:
							mouseUp(i.p);
							break;
						case ShiftMouseDown:
							shiftMouseDown(i.p);
							break;
					}
				}
			}
		}
		
		step(settings);
	}
	
	private final Color3f color1 = new Color3f(.3f, .95f, .3f);
	private final Color3f color2 = new Color3f(.3f, .3f, .95f);
	private final Color3f color3 = new Color3f(.9f, .9f, .9f);
	private final Color3f color4 = new Color3f(.6f, .61f, 1);
	private final Color3f mouseColor = new Color3f(0f, 1f, 0f);
	private final Vec2 p1 = new Vec2();
	private final Vec2 p2 = new Vec2();
	
	
	
	public void step(TestbedSettings settings){
		float timeStep = settings.hz > 0f ? 1f/settings.hz : 0;
		
		if(settings.singleStep && !settings.pause){
			settings.pause = true;
		}
		
		if(settings.pause){
			if(settings.singleStep){
				settings.singleStep = false;
			}else{
				timeStep = 0;
			}
			
			m_debugDraw.drawString(5, m_textLine, "****PAUSED****", Color3f.WHITE);
			m_textLine += 15;
		}
		
		int flags = 0;
		flags += settings.drawShapes? DebugDraw.e_shapeBit : 0;
		flags += settings.drawJoints? DebugDraw.e_jointBit: 0;
		flags += settings.drawAABBs? DebugDraw.e_aabbBit: 0;
		flags += settings.drawPairs? DebugDraw.e_pairBit: 0;
		flags += settings.drawCOMs?	 DebugDraw.e_centerOfMassBit: 0;
		flags += settings.drawDynamicTree? DebugDraw.e_dynamicTreeBit: 0;
		m_debugDraw.setFlags(flags);
		
		m_world.setWarmStarting(settings.enableWarmStarting);
		m_world.setContinuousPhysics(settings.enableContinuous);
		
		m_pointCount = 0;
		
		m_world.step(timeStep, settings.velocityIterations, settings.positionIterations);
		
		m_world.drawDebugData();
		
		if(timeStep > 0f){
			++m_stepCount;
		}
		
		if(settings.drawStats){
//			Vec2.watchCreations = true;
			m_debugDraw.drawString(5, m_textLine, "Engine Info", color4);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Framerate: "+ panel.getCalculatedFrameRate(), Color3f.WHITE);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine,"bodies/contacts/joints/proxies = "+m_world.getBodyCount()+"/"+m_world.getContactCount()+"/"+m_world.getJointCount()+"/"+m_world.getProxyCount(), Color3f.WHITE);
//			m_textLine += 20;
//			m_debugDraw.drawString(5, m_textLine, "Pooling Info", color4);
//			m_textLine += 15;
//			m_debugDraw.drawString(5, m_textLine, "Vec2 creations: "+ Vec2.creationCount, Color3f.WHITE);
//			m_textLine += 15;
//			m_debugDraw.drawString(5, m_textLine, "Contact pooled/active: "+ Contact.contactPoolCount+"/"+Contact.activeContacts, Color3f.WHITE);
			m_textLine += 20;

//			Vec2.creationCount = 0;
		}else{
//			Vec2.watchCreations = false;
		}
		
		if(settings.drawHelp){
			m_debugDraw.drawString(5, m_textLine, "Help", color4);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Click and drag the left mouse button to move objects.", Color3f.WHITE);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Shift-Click to aim a bullet, or press space.", Color3f.WHITE);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Click and drag the right mouse button to move the view.", Color3f.WHITE);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Scroll to zoom in/out.", Color3f.WHITE);
			m_textLine += 15;
			m_debugDraw.drawString(5, m_textLine, "Press '[' or ']' to change tests, and 'r' to restart.", Color3f.WHITE);
			m_textLine += 20;
		}
		
		if(!textList.isEmpty()){
			m_debugDraw.drawString(5, m_textLine, "Test Info", color4);
			m_textLine += 15;
			for(String s : textList){
				m_debugDraw.drawString(5, m_textLine, s, Color3f.WHITE);
				m_textLine+=15;
			}
			textList.clear();
		}
		
		if(m_mouseJoint != null){
			m_mouseJoint.getAnchorB(p1);
			Vec2 p2 = m_mouseJoint.getTarget();
			
			m_debugDraw.drawSegment(p1, p2, mouseColor);
		}
		
		if(bombSpawning){
			m_debugDraw.drawSegment(bombSpawnPoint, m_mouseWorld, Color3f.WHITE);
		}
		
		if(settings.drawContactPoints){
			final float axisScale = .3f;
			
			for(int i=0; i<m_pointCount; i++){
				
				ContactPoint point = points[i];
				
				if(point.state == PointState.ADD_STATE){
					m_debugDraw.drawPoint(point.position, 10f, color1);
				}
				else if(point.state == PointState.PERSIST_STATE){
					m_debugDraw.drawPoint(point.position, 5f, color2);
				}
				
				if(settings.drawContactNormals){
					p1.set(point.position);
					p2.set(point.normal).mulLocal(axisScale).addLocal(p1);
					m_debugDraw.drawSegment(p1, p2, color3);
				}
			}
		}
	}
	
	public void queueShiftMouseDown(Vec2 p){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.ShiftMouseDown, p));
		}
	}
	
	public void queueMouseUp(Vec2 p){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.MouseUp, p));
		}
	}
	
	public void queueMouseDown(Vec2 p){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.MouseDown, p));
		}
	}
	
	public void queueMouseMove(Vec2 p){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.MouseMove, p));
		}
	}
	
	public void queueKeyPressed(char c, int code){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.KeyPressed, c, code));
		}
	}
	
	public void queueKeyReleased(char c, int code){
		synchronized (inputQueue) {
			inputQueue.addLast(new QueueItem(QueueItemType.KeyReleased, c, code));
		}
	}
	
	
	public void shiftMouseDown(Vec2 p){
		m_mouseWorld.set(p);
		
		if(m_mouseJoint != null){
			return;
		}
		
		spawnBomb(p);
	}
	
	public void mouseUp(Vec2 p){
		if(m_mouseJoint != null){
			m_world.destroyJoint(m_mouseJoint);
			m_mouseJoint = null;
		}
		
		if(bombSpawning){
			completeBombSpawn(p);
		}
	}
	
	private final AABB queryAABB = new AABB();
	private final TestQueryCallback callback = new TestQueryCallback();
	
	public void mouseDown(Vec2 p){
		m_mouseWorld.set(p);
		
		if(m_mouseJoint != null){
			return;
		}
		
		queryAABB.lowerBound.set(p.x - .001f, p.y - .001f);
		queryAABB.upperBound.set(p.x + .001f, p.y + .001f);
		callback.point.set(p);
		callback.fixture = null;
		m_world.queryAABB(callback, queryAABB);
		
		if(callback.fixture != null){
			Body body = callback.fixture.getBody();
			MouseJointDef def = new MouseJointDef();
			def.bodyA = m_groundBody;
			def.bodyB = body;
			def.target.set(p);
			def.maxForce = 1000f * body.getMass();
			m_mouseJoint = (MouseJoint) m_world.createJoint(def);
			body.setAwake(true);
		}
	}
	
	public void mouseMove(Vec2 p){
		m_mouseWorld.set(p);
		
		if(m_mouseJoint != null){
			m_mouseJoint.setTarget(p);
		}
	}
	
	public void setTitle(String argTitle){
		title = argTitle;
	}
	
	public void addTextLine(String argTextLine){
		textList.add(argTextLine);
	}
	
	private final Vec2 p = new Vec2();
	private final Vec2 v = new Vec2();
	
	public void lanchBomb(){
		p.set((float)(Math.random()*30 - 15), 30f);
		v.set(p).mulLocal(-5f);
		launchBomb(p, v);
	}
	
	private final AABB aabb = new AABB();
	
	public void launchBomb(Vec2 position, Vec2 velocity){
		if(m_bomb != null){
			m_world.destroyBody(m_bomb);
			m_bomb = null;
		}
		// todo optimize this
		BodyDef bd = new BodyDef();
		bd.type = BodyType.DYNAMIC;
		bd.position.set(position);
		bd.bullet = true;
		m_bomb = m_world.createBody(bd);
		m_bomb.setLinearVelocity(velocity);
		
		CircleShape circle = new CircleShape();
		circle.m_radius = 0.3f;
		
		FixtureDef fd = new FixtureDef();
		fd.shape = circle;
		fd.density = 20f;
		fd.restitution = 0;
		
		Vec2 minV = new Vec2(position);
		Vec2 maxV = new Vec2(position);
		
		minV.subLocal(new Vec2(.3f,.3f));
		maxV.addLocal(new Vec2(.3f, .3f));
		
		aabb.lowerBound.set(minV);
		aabb.upperBound.set(maxV);
		
		m_bomb.createFixture(fd);
	}
	
	public void spawnBomb(Vec2 worldPt){
		bombSpawnPoint.set(worldPt);
		bombSpawning = true;
	}
	
	private final Vec2 vel = new Vec2();
	
	public void completeBombSpawn( Vec2 p){
		if(bombSpawning == false){
			return;
		}
		
		float multiplier = 30f;
		vel.set(bombSpawnPoint).subLocal(p);
		vel.mulLocal(multiplier);
		launchBomb(bombSpawnPoint, vel);
		bombSpawning = false;
	}
	
	public void jointDestroyed(Joint joint){
		
	}
	
	/**
	 * @see org.jbox2d.callbacks.ContactListener#beginContact(org.jbox2d.dynamics.contacts.Contact)
	 */
	public void beginContact(Contact contact) { }
	/**
	 * @see org.jbox2d.callbacks.ContactListener#endContact(org.jbox2d.dynamics.contacts.Contact)
	 */
	public void endContact(Contact contact) { }
	/**
	 * @see org.jbox2d.callbacks.ContactListener#postSolve(org.jbox2d.dynamics.contacts.Contact, org.jbox2d.callbacks.ContactImpulse)
	 */
	public void postSolve(Contact contact, ContactImpulse impulse) { }
	
	private final PointState[] state1 = new PointState[Settings.maxManifoldPoints];
	private final PointState[] state2 = new PointState[Settings.maxManifoldPoints];
	private final WorldManifold worldManifold = new WorldManifold();
	/**
	 * @see org.jbox2d.callbacks.ContactListener#preSolve(org.jbox2d.dynamics.contacts.Contact, org.jbox2d.collision.Manifold)
	 */
	public void preSolve(Contact contact, Manifold oldManifold) {
		Manifold manifold = contact.getManifold();
		
		if(manifold.pointCount == 0){
			return;
		}
		
		Fixture fixtureA = contact.getFixtureA();
		Fixture fixtureB = contact.getFixtureB();
		
		Collision.getPointStates(state1, state2, oldManifold, manifold);
		
		contact.getWorldManifold(worldManifold);
		
		for(int i=0; i<manifold.pointCount && m_pointCount < MAX_CONTACT_POINTS; i++){
			ContactPoint cp = points[m_pointCount];
			cp.fixtureA = fixtureA;
			cp.fixtureB = fixtureB;
			cp.position.set(worldManifold.points[i]);
			cp.normal.set(worldManifold.normal);
			cp.state = state2[i];
			++m_pointCount;
		}
	}

	public void keyPressed(char argKeyChar, int argKeyCode) {}
	
	public void keyReleased(char argKeyChar, int argKeyCode) {}
}

class TestQueryCallback implements QueryCallback{

	public final Vec2 point;
	public Fixture fixture;
	
	public TestQueryCallback(){
		point = new Vec2();
		fixture = null;
	}
	
	/**
	 * @see org.jbox2d.callbacks.QueryCallback#reportFixture(org.jbox2d.dynamics.Fixture)
	 */
	public boolean reportFixture(Fixture argFixture) {
		Body body = argFixture.getBody();
		if(body.getType() == BodyType.DYNAMIC){
			boolean inside = argFixture.testPoint(point);
			if(inside){
				fixture = argFixture;
				
				return false;
			}
		}
		
		return true;
	}
}

enum QueueItemType {
	MouseDown, MouseMove, MouseUp, ShiftMouseDown, KeyPressed, KeyReleased
}

class QueueItem{
	public QueueItemType type;
	public Vec2 p;
	public char c;
	public int code;
	
	public QueueItem(QueueItemType t, Vec2 pt){
		type = t;
		p = pt;
	}
	
	public QueueItem(QueueItemType t, char cr, int cd){
		type = t;
		c = cr;
		code = cd;
	}
}
