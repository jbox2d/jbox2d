/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
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
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
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

  private final Vec2 dragginMouse = new Vec2();
  private boolean drag = false;

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

    addMouseListener(new MouseAdapter() {
      @Override
      public void mousePressed(MouseEvent e) {
        dragginMouse.set(e.getX(), e.getY());
        drag = e.getButton() == MouseEvent.BUTTON3;
      }
    });

    addMouseMotionListener(new MouseMotionAdapter() {
      @Override
      public void mouseDragged(MouseEvent e) {
        if (!drag) {
          return;
        }
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }
        Vec2 diff = new Vec2(e.getX(), e.getY());
        diff.subLocal(dragginMouse);
        currTest.getDebugDraw().getViewportTranform().getScreenVectorToWorld(diff, diff);
        currTest.getDebugDraw().getViewportTranform().getCenter().subLocal(diff);

        dragginMouse.set(e.getX(), e.getY());
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

  public boolean render() {
    if (dbImage == null) {
      log.debug("dbImage is null, creating a new one");
      if (panelWidth <= 0 || panelHeight <= 0) {
        return false;
      }
      dbImage = createImage(panelWidth, panelHeight);
      if (dbImage == null) {
        log.error("dbImage is still null, ignoring render call");
        return false;
      }
      dbg = (Graphics2D) dbImage.getGraphics();
    }
    dbg.setColor(Color.black);
    dbg.fillRect(0, 0, panelWidth, panelHeight);
    return true;
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
