/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: * Redistributions of source code must retain the
 * above copyright notice, this list of conditions and the following disclaimer. * Redistributions
 * in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.jbox2d.testbed.framework.j2d;

import java.awt.AWTError;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.List;

import javax.swing.JPanel;

import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedCamera.ZoomType;
import org.jbox2d.testbed.framework.TestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedPanel;
import org.jbox2d.testbed.framework.TestbedTest;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Lists;

/**
 * @author Daniel Murphy
 */
@SuppressWarnings("serial")
public class TestPanelJ2D extends JPanel implements TestbedPanel {
  private static final Logger log = LoggerFactory.getLogger(TestPanelJ2D.class);

  public static final int SCREEN_DRAG_BUTTON = 3;

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private Graphics2D dbg = null;
  private Image dbImage = null;

  private int panelWidth;
  private int panelHeight;

  private final TestbedController controller;

  private final Vec2 oldDragMouse = new Vec2();

  private final Vec2 mouse = new Vec2();

  public TestPanelJ2D(final TestbedModel model, final TestbedController controller) {
    this.controller = controller;
    setBackground(Color.black);
    setPreferredSize(new Dimension(INIT_WIDTH, INIT_HEIGHT));
    updateSize(INIT_WIDTH, INIT_HEIGHT);

    List<String> help = Lists.newArrayList();
    help.add("Click and drag the left mouse button to move objects.");
    help.add("Shift-Click to aim a bullet, or press space.");
    help.add("Click and drag the right mouse button to move the view.");
    help.add("Scroll to zoom in/out.");
    help.add("Press '[' or ']' to change tests, and 'r' to restart.");
    model.setImplSpecificHelp(help);

    addMouseWheelListener(new MouseWheelListener() {
      public void mouseWheelMoved(MouseWheelEvent e) {
        int notches = e.getWheelRotation();
        TestbedTest currTest = model.getCurrTest();
        if (currTest == null) {
          return;
        }
        ZoomType zoom = notches < 0 ? ZoomType.ZOOM_IN : ZoomType.ZOOM_OUT;
        currTest.getCamera().zoomToPoint(mouse, zoom);
      }
    });

    addMouseListener(new MouseAdapter() {
      @Override
      public void mouseReleased(MouseEvent arg0) {
        controller.queueMouseUp(new Vec2(arg0.getX(), arg0.getY()), arg0.getButton());
        
        if (model.getCodedKeys()[KeyEvent.VK_SHIFT]) {
          controller.queueMouseUp(new Vec2(arg0.getX(), arg0.getY()), 10);
        }
      }

      @Override
      public void mousePressed(MouseEvent arg0) {
        controller.queueMouseDown(new Vec2(arg0.getX(), arg0.getY()), arg0.getButton());

        if (arg0.getButton() == SCREEN_DRAG_BUTTON) {
          oldDragMouse.set(arg0.getX(), arg0.getY());
        }
        if (model.getCodedKeys()[KeyEvent.VK_SHIFT]) {
          controller.queueMouseDown(new Vec2(arg0.getX(), arg0.getY()), 10);
        }
      }
    });

    addMouseMotionListener(new MouseMotionAdapter() {
      @Override
      public void mouseMoved(MouseEvent arg0) {
        mouse.set(arg0.getX(), arg0.getY());
        controller.queueMouseMove(new Vec2(mouse));
      }

      @Override
      public void mouseDragged(MouseEvent arg0) {
        mouse.set(arg0.getX(), arg0.getY());
        controller.queueMouseDrag(new Vec2(mouse), arg0.getButton());

        if (arg0.getButton() == SCREEN_DRAG_BUTTON) {
          TestbedTest currTest = model.getCurrTest();
          if (currTest == null) {
            return;
          }
          Vec2 diff = oldDragMouse.sub(mouse);
          currTest.getCamera().moveWorld(diff);
          oldDragMouse.set(mouse);
        }
        if (model.getCodedKeys()[KeyEvent.VK_SHIFT]) {
          controller.queueMouseDrag(new Vec2(arg0.getX(), arg0.getY()), 10);
        }
      }
    });

    addComponentListener(new ComponentAdapter() {
      @Override
      public void componentResized(ComponentEvent e) {
        updateSize(getWidth(), getHeight());
        dbImage = null;
      }
    });

    addKeyListener(new KeyAdapter() {
      @Override
      public void keyReleased(KeyEvent arg0) {
        controller.queueKeyReleased(arg0.getKeyChar(), arg0.getKeyCode());
      }

      @Override
      public void keyPressed(KeyEvent arg0) {
        char c = arg0.getKeyChar();
        controller.queueKeyPressed(c, arg0.getKeyCode());
        switch (c) {
          case '[':
            controller.lastTest();
            break;
          case ']':
            controller.nextTest();
            break;
          case 'r':
            controller.reset();
            break;
          case ' ':
            controller.queueLaunchBomb();
            break;
        }
      }
    });
  }

  public Graphics2D getDBGraphics() {
    return dbg;
  }

  private void updateSize(int width, int height) {
    panelWidth = width;
    panelHeight = height;
    controller.updateExtents(width / 2, height / 2);
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
      dbg.setFont(new Font("Courier New", Font.PLAIN, 12));
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
