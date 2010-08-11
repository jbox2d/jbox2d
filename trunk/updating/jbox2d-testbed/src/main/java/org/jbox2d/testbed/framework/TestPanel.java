/**
 * Created at 3:13:48 AM Jul 17, 2010
 */
package org.jbox2d.testbed.framework;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

import javax.swing.JPanel;

import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.Manifold;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.contacts.Contact;

/**
 * @author Daniel Murphy
 */
public class TestPanel extends JPanel implements Runnable{
	
	public volatile static boolean[] keys = new boolean[256];
	public volatile static boolean[] codedKeys = new boolean[512];
	
	private TestbedTest currTest = null;
	private TestbedTest nextTest = null;
	
	public DebugDraw draw = new DebugDrawJ2D(this);
	
	public final Vec2 mouse = new Vec2();
	public int mouseButton = -1;
	public final TestbedSettings settings;
	
	private long startTime;
	private long frameCount;
	private int targetFrameRate;
	private float frameRate = 0;
	private boolean animating = false;
	private Graphics2D dbg = null;
	private Image dbImage = null;
	private Thread animator;

	
	public TestPanel(TestbedSettings argSettings) {
		setBackground(Color.black);
		setPreferredSize(new Dimension(600,500));
		settings = argSettings;
		setFrameRate(60);
		animator = new Thread(this, "Animation Thread");
	}
	
	public Graphics2D getDBGraphics(){
		return dbg;
	}
	
	public void init(){
		grabFocus();
		
		addKeyListener(new KeyListener() {
			public void keyTyped(KeyEvent e) {}
			
			public void keyReleased(KeyEvent e) {
				char key = e.getKeyChar();
				int code = e.getKeyCode();
				if(key != KeyEvent.CHAR_UNDEFINED){
					keys[key] = false;
				}
				codedKeys[code] = false;
			}
			
			public void keyPressed(KeyEvent e) {
				char key = e.getKeyChar();
				int code = e.getKeyCode();
				if(key != KeyEvent.CHAR_UNDEFINED){
					keys[key] = true;
				}
				codedKeys[code] = true;
				if(currTest != null){
					currTest.keyPressed(key, code);
				}
			}
		});
		addMouseMotionListener(new MouseMotionListener() {
			
			private final Vec2 posDif = new Vec2();
			public void mouseDragged(MouseEvent e) {
				if(mouseButton == MouseEvent.BUTTON3){
					posDif.set(mouse);
					mouse.set(e.getX(), e.getY());
					posDif.subLocal(mouse);
					draw.getViewportTranform().vectorInverseTransform(posDif, posDif);
					draw.getViewportTranform().getCenter().addLocal(posDif);
					if(currTest != null){
						currTest.cachedCameraX = draw.getViewportTranform().getCenter().x;
						currTest.cachedCameraY = draw.getViewportTranform().getCenter().y;
					}
				}
				if(currTest != null){
					pos.set(e.getX(), e.getY());
					mouse.set(pos);
					draw.getScreenToWorldToOut(pos, pos);
					currTest.mouseMove(pos);
				}
			}
			
			private final Vec2 pos = new Vec2();
			public void mouseMoved(MouseEvent e) {
				if(currTest != null){
					pos.set(e.getX(), e.getY());
					mouse.set(pos);
					draw.getScreenToWorldToOut(pos, pos);
					currTest.mouseMove(pos);
				}
			}
		});
		
		addMouseListener(new MouseAdapter() {
			private final Vec2 pos = new Vec2();
			public void mouseReleased(MouseEvent e) {
				if(currTest != null){
					pos.x = e.getX();
					pos.y = e.getY();
					mouse.set(pos);
					draw.getScreenToWorldToOut(pos, pos);
					currTest.mouseUp(pos);
				}
			}
			
			private final Vec2 pos2 = new Vec2();
			public void mousePressed(MouseEvent e) {
				grabFocus();
				if(currTest != null){
					pos2.x = e.getX();
					pos2.y = e.getY();
					mouse.set(pos2);
					mouseButton = e.getButton();
					if(e.getButton() == MouseEvent.BUTTON1){
						draw.getScreenToWorldToOut(pos2, pos2);
						if(codedKeys[KeyEvent.VK_SHIFT]){
							currTest.shiftMouseDown(pos2);
						}else{
							currTest.mouseDown(pos2);
						}
					}
				}
			}
		});
		
		addMouseWheelListener(new MouseWheelListener() {
			
			private final Vec2 oldCenter = new Vec2();
			private final Vec2 newCenter = new Vec2();
			private final Mat22 upScale = Mat22.createScaleTransform( 1.05f);
			private final Mat22 downScale = Mat22.createScaleTransform( .95f);
			public void mouseWheelMoved(MouseWheelEvent e) {
				DebugDraw d = draw;
        		int notches = e.getWheelRotation();
        		
            	OBBViewportTransform trans = (OBBViewportTransform) d.getViewportTranform();
            	oldCenter.set(currTest.mouseWorld);
            	//Change the zoom and clamp it to reasonable values - can't clamp now.
            	if (notches < 0) {
            		trans.mulByTransform( upScale);
            		currTest.cachedCameraScale *= 1.05;
            	}
            	else if (notches > 0) {
            		trans.mulByTransform( downScale);
            		currTest.cachedCameraScale *= .95f;
            	}
            	
            	d.getScreenToWorldToOut(mouse, newCenter);
            	
            	
            	Vec2 transformedMove = oldCenter.subLocal(newCenter);
            	d.getViewportTranform().setCenter( d.getViewportTranform().getCenter().addLocal(transformedMove));

            	currTest.cachedCameraX = d.getViewportTranform().getCenter().x;
            	currTest.cachedCameraY = d.getViewportTranform().getCenter().y;
			}
		});
		
		if(currTest != null){
			currTest.init(draw);
		}
	}
	
	public void changeTest(TestbedTest test){
		nextTest = test;
	}
	
	public void update(){
		if(currTest != null){
			currTest.step(settings);
		}
	}	
	
	public void setFrameRate(int fps) {
		if (fps == 0) {
			System.err.println("Animation: fps cannot be zero.  Setting looping to false");
		}
		
		if (fps < 0) {
			System.err.println("Animation: fps cannot be negative.  Re-assigning fps to default " + 30
					+ " fps.");
			fps = 30;
		}
		targetFrameRate = fps;
		frameRate = fps;
	}
	
	public int getFrameRate() {
		return targetFrameRate;
	}
	
	public float getCalculatedFrameRate() {
		return frameRate;
	}
	
	public long getStartTime() {
		return startTime;
	}
	
	public long getFrameCount() {
		return frameCount;
	}
	
	public boolean isAnimating() {
		return animating;
	}
	
	public void start() {
		if (animating != true) {
			frameCount = 0;
			animator.start();
		}
		else {
			System.err.println("\nAnimation is already animating.");
			System.err.println("\n");
		}
	}
	
	public void stop() {
		animating = false;
	}
	
	public void render(){
		if(dbImage == null){
			dbImage = createImage(600, 500);
			if(dbImage == null){
				System.err.println("dbImage is null");
				return;
			}
			dbg = (Graphics2D)dbImage.getGraphics();
		}
		dbg.setColor(Color.black);
		dbg.fillRect(0, 0, 600, 500);
	}
	
	public void paintScreen(){
		Graphics g;
		try{
			g = this.getGraphics();
			if((g != null) && dbImage != null){
				g.drawImage(dbImage, 0, 0, null);
				Toolkit.getDefaultToolkit().sync();
				g.dispose();
			}
		}catch(Exception e){
			System.err.println("Graphics context error: "+ e);
		}
	}
	
	/**
	 * @see javax.swing.JComponent#addNotify()
	 */
	@Override
	public void addNotify() {
		super.addNotify();
		start();
	}
	
	public void run() { // animation loop
		long beforeTime, afterTime, updateTime, timeDiff, sleepTime, timeSpent;
		init();
		float timeInSecs;
		beforeTime = startTime = updateTime = System.nanoTime();
		sleepTime = 0;
		
		animating = true;
		while (animating) {
			
			if(nextTest != null){
				currTest = nextTest;
				currTest.setPanel(this);
				currTest.init(draw);
				nextTest = null;
			}
			
			timeSpent = beforeTime - updateTime;
			if (timeSpent > 0) {
				timeInSecs = timeSpent * 1.0f / 1000000000.0f;
				updateTime = System.nanoTime();
				frameRate = (frameRate * 0.9f)
						+ (1.0f / timeInSecs ) * 0.1f;
			}else{
				updateTime = System.nanoTime();
			}
			
			render();
			update();
			paintScreen();
			frameCount++;
			
			afterTime = System.nanoTime();
			
			timeDiff = afterTime - beforeTime;
			sleepTime = (1000000000/targetFrameRate - timeDiff)/1000000;
			//sleepTime = 1000 / targetFrameRate - timeDiff / 1000000;
			if (sleepTime > 0) {
				try {
					Thread.sleep(sleepTime);
				}
				catch (InterruptedException ex) {}
			}
			
			beforeTime = System.nanoTime();
			
		} // end of run loop
	}
}
