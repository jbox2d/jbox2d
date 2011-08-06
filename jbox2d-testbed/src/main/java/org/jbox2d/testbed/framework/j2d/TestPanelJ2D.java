package org.jbox2d.testbed.framework.j2d;

import java.awt.AWTError;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestPanelJ2D extends JPanel implements TestbedPanel {
  private static final Logger log = LoggerFactory.getLogger(TestPanelJ2D.class);

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private static final float ZOOM_OUT_SCALE = .95f;
  private static final float ZOOM_IN_SCALE = 1.05f;

  private Graphics2D dbg = null;
  private Image dbImage = null;

  private int panelWidth;
  private int panelHeight;

  private final TestbedModel model;
  private final DebugDrawJ2D draw;

  public TestPanelJ2D(TestbedModel argModel) {
    setBackground(Color.black);
    draw = new DebugDrawJ2D(this);
    model = argModel;
    updateSize(INIT_WIDTH, INIT_HEIGHT);
    setPreferredSize(new Dimension(INIT_WIDTH, INIT_HEIGHT));

    addMouseWheelListener(new MouseWheelListener() {

      private final Vec2 oldCenter = new Vec2();
      private final Vec2 newCenter = new Vec2();
      private final Mat22 upScale = Mat22.createScaleTransform(ZOOM_IN_SCALE);
      private final Mat22 downScale = Mat22.createScaleTransform(ZOOM_OUT_SCALE);

      public void mouseWheelMoved(MouseWheelEvent e) {
        DebugDraw d = draw;
        int notches = e.getWheelRotation();
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }
        OBBViewportTransform trans = (OBBViewportTransform) d.getViewportTranform();
        oldCenter.set(model.getCurrTest().getWorldMouse());
        // Change the zoom and clamp it to reasonable values - can't clamp now.
        if (notches < 0) {
          trans.mulByTransform(upScale);
          currTest.setCachedCameraScale(currTest.getCachedCameraScale() * ZOOM_IN_SCALE);
        } else if (notches > 0) {
          trans.mulByTransform(downScale);
          currTest.setCachedCameraScale(currTest.getCachedCameraScale() * ZOOM_OUT_SCALE);
        }

        d.getScreenToWorldToOut(model.getMouse(), newCenter);

        Vec2 transformedMove = oldCenter.subLocal(newCenter);
        d.getViewportTranform().setCenter(
            d.getViewportTranform().getCenter().addLocal(transformedMove));

        currTest.setCachedCameraPos(d.getViewportTranform().getCenter());
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

  @Override
  public DebugDraw getDebugDraw() {
    return draw;
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  private void updateSize(int argWidth, int argHeight) {
    panelWidth = argWidth;
    panelHeight = argHeight;
    draw.getViewportTranform().setExtents(argWidth / 2, argHeight / 2);
  }

  public void render() {
    if (dbImage == null) {
      log.debug("dbImage is null, creating a new one");
      if (panelWidth <= 0 || panelHeight <= 0) {
        return;
      }
      dbImage = createImage(panelWidth, panelHeight);
      if (dbImage == null) {
        log.error("dbImage is still null, ignoring render call");
        return;
      }
      dbg = (Graphics2D) dbImage.getGraphics();
    }
    dbg.setColor(Color.black);
    dbg.fillRect(0, 0, panelWidth, panelHeight);
  }

  public void paintScreen() {
    try {
      Graphics g = this.getGraphics();
      if ((g != null) && dbImage != null) {
        g.drawImage(dbImage, 0, 0, null);
        Toolkit.getDefaultToolkit().sync();
        g.dispose();
      }
    } catch (AWTError e) {
      log.error("Graphics context error", e);
    }
  }
}
