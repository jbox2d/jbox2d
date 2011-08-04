package org.jbox2d.testbed.framework.j2d;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

import javax.swing.JPanel;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedPanel;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.testbed.framework.TestbedModel.TestChangedListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestPanelJ2D extends JPanel implements Runnable, TestbedPanel, TestChangedListener {
  private static final Logger log = LoggerFactory.getLogger(TestPanelJ2D.class);
  
  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private TestbedTest currTest = null;
  private TestbedTest nextTest = null;

  private long startTime;
  private long frameCount;
  private int targetFrameRate;
  private float frameRate = 0;
  private boolean animating = false;
  private Graphics2D dbg = null;
  private Image dbImage = null;
  private Thread animator;

  private int panelWidth;
  private int panelHeight;

  private final TestbedModel model;
  private final DebugDrawJ2D draw;
  private boolean autoStart;

  public TestPanelJ2D(TestbedModel argModel, boolean argAutoStart) {
    setBackground(Color.black);
    autoStart = argAutoStart;
    draw = new DebugDrawJ2D(this);
    model = argModel;
    model.setDebugDraw(draw);
    updateSize(INIT_WIDTH, INIT_HEIGHT);
    setFrameRate(60);
    animator = new Thread(this, "Animation Thread");
    model.addTestChangeListener(this);
    
    addMouseWheelListener(new MouseWheelListener() {

      private final Vec2 oldCenter = new Vec2();
      private final Vec2 newCenter = new Vec2();
      private final Mat22 upScale = Mat22.createScaleTransform(1.05f);
      private final Mat22 downScale = Mat22.createScaleTransform(.95f);

      public void mouseWheelMoved(MouseWheelEvent e) {
        DebugDraw d = draw;
        int notches = e.getWheelRotation();

        OBBViewportTransform trans = (OBBViewportTransform) d.getViewportTranform();
        oldCenter.set(currTest.getWorldMouse());
        // Change the zoom and clamp it to reasonable values - can't clamp now.
        if (notches < 0) {
          trans.mulByTransform(upScale);
          currTest.cachedCameraScale *= 1.05;
        } else if (notches > 0) {
          trans.mulByTransform(downScale);
          currTest.cachedCameraScale *= .95f;
        }

        d.getScreenToWorldToOut(model.getMouse(), newCenter);

        Vec2 transformedMove = oldCenter.subLocal(newCenter);
        d.getViewportTranform().setCenter(
            d.getViewportTranform().getCenter().addLocal(transformedMove));

        currTest.cachedCameraX = d.getViewportTranform().getCenter().x;
        currTest.cachedCameraY = d.getViewportTranform().getCenter().y;
      }
    });

    addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        updateSize(getWidth(), getHeight());
        dbImage = null;
      }
    });
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  public void updateSize(int argWidth, int argHeight) {
    panelWidth = argWidth;
    panelHeight = argHeight;
    draw.getViewportTranform().setExtents(argWidth / 2, argHeight / 2);
    setPreferredSize(new Dimension(panelWidth, panelHeight));
  }

  public TestbedTest getTest() {
    return currTest;
  }

  private void loopInit() {
    grabFocus();
    
    if (currTest != null) {
      currTest.init(model);
    }
  }

  @Override
  public void testChanged(TestbedTest argTest) {
    nextTest = argTest;
    grabFocus();
  }

  public void update() {
    if (currTest != null) {
      currTest.update();
    }
  }

  @Override
  public void setFrameRate(int fps) {
    if (fps == 0) {
      log.warn("Animation: fps cannot be zero.  Setting looping to false");
    }

    if (fps < 0) {
      log.warn("Animation: fps cannot be negative.  Re-assigning fps to default " + 30
          + " fps.");
      fps = 30;
    }
    targetFrameRate = fps;
    frameRate = fps;
  }

  @Override
  public int getFrameRate() {
    return targetFrameRate;
  }

  @Override
  public float getCalculatedFrameRate() {
    return frameRate;
  }

  @Override
  public long getStartTime() {
    return startTime;
  }

  @Override
  public long getFrameCount() {
    return frameCount;
  }

  @Override
  public boolean isAnimating() {
    return animating;
  }

  public void start() {
    if (animating != true) {
      frameCount = 0;
      animator.start();
    } else {
      log.warn("Animation is already animating.");
    }
  }

  public void stop() {
    animating = false;
  }

  private void render() {
    if (dbImage == null) {
      if (panelWidth <= 0 || panelHeight <= 0) {
        return;
      }
      dbImage = createImage(panelWidth, panelHeight);
      if (dbImage == null) {
        System.err.println("dbImage is null");
        return;
      }
      dbg = (Graphics2D) dbImage.getGraphics();
    }
    dbg.setColor(Color.black);
    dbg.fillRect(0, 0, panelWidth, panelHeight);
  }

  private void paintScreen() {
    Graphics g;
    try {
      g = this.getGraphics();
      if ((g != null) && dbImage != null) {
        g.drawImage(dbImage, 0, 0, null);
        Toolkit.getDefaultToolkit().sync();
        g.dispose();
      }
    } catch (Exception e) {
      System.err.println("Graphics context error: " + e);
    }
  }

  @Override
  public void addNotify() {
    super.addNotify();
    if(autoStart){
      start();
    }
  }

  public void run() {
    long beforeTime, afterTime, updateTime, timeDiff, sleepTime, timeSpent;
    float timeInSecs;
    beforeTime = startTime = updateTime = System.nanoTime();
    sleepTime = 0;

    animating = true;
    loopInit();
    while (animating) {

      if (nextTest != null) {
        currTest = nextTest;
        currTest.setPanel(this);
        currTest.init(model);
        nextTest = null;
      }

      timeSpent = beforeTime - updateTime;
      if (timeSpent > 0) {
        timeInSecs = timeSpent * 1.0f / 1000000000.0f;
        updateTime = System.nanoTime();
        frameRate = (frameRate * 0.9f) + (1.0f / timeInSecs) * 0.1f;
      } else {
        updateTime = System.nanoTime();
      }

      render();
      update();
      paintScreen();
      frameCount++;

      afterTime = System.nanoTime();

      timeDiff = afterTime - beforeTime;
      sleepTime = (1000000000 / targetFrameRate - timeDiff) / 1000000;
      if (sleepTime > 0) {
        try {
          Thread.sleep(sleepTime);
        } catch (InterruptedException ex) {
        }
      }

      beforeTime = System.nanoTime();
    } // end of run loop
  }
}
