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
package org.jbox2d.testbed.framework.jogl;

import java.awt.Dimension;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLCapabilities;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.GLProfile;
import javax.media.opengl.awt.GLJPanel;
import javax.media.opengl.glu.GLU;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Vec2;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedPanel;
import org.jbox2d.testbed.framework.TestbedTest;

public class JoglPanel extends GLJPanel implements TestbedPanel, GLEventListener {
  private static final long serialVersionUID = 1L;
  
  private static final float ZOOM_OUT_SCALE = .95f;
  private static final float ZOOM_IN_SCALE = 1.05f;
  
  private final JoglDebugDraw draw;
  private final TestbedModel model;

  public JoglPanel(TestbedModel argModel) {
    super(new GLCapabilities(GLProfile.getDefault()));
    model = argModel;
    draw = new JoglDebugDraw(this);
    setSize(600, 600);
    setPreferredSize(new Dimension(600, 600));
    setAutoSwapBufferMode(true);
    addGLEventListener(this);
    
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
  }

  @Override
  public DebugDraw getDebugDraw() {
    return draw;
  }

  @Override
  public boolean render() {
    return true;
  }

  @Override
  public void paintScreen() {
    display();
  }

  @Override
  public void display(GLAutoDrawable arg0) {
    getGL().getGL2().glClear(GL2.GL_COLOR_BUFFER_BIT);
    
    if (model.getCurrTest() != null) {
      model.getRunningTest().update();
    }
    
    getGL().glFlush();
  }

  @Override
  public void dispose(GLAutoDrawable arg0) {
    // TODO Auto-generated method stub

  }

  @Override
  public void init(GLAutoDrawable arg0) {
    getGL().getGL2().glLineWidth(1f);
    getGL().getGL2().glEnable(GL2.GL_BLEND);
    getGL().getGL2().glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
  }

  @Override
  public void reshape(GLAutoDrawable arg0, int arg1, int arg2, int arg3, int arg4) {
    GL2 gl2 = arg0.getGL().getGL2();
    
    gl2.glMatrixMode( GL2.GL_PROJECTION );
    gl2.glLoadIdentity();

    // coordinate system origin at lower left with width and height same as the window
    GLU glu = new GLU();
    glu.gluOrtho2D( 0.0f, getWidth(), 0.0f, getHeight() );

    gl2.glMatrixMode( GL2.GL_MODELVIEW );
    gl2.glLoadIdentity();

    gl2.glViewport( 0, 0, getWidth(), getHeight() );

    draw.getViewportTranform().setExtents(arg3 / 2, arg4 / 2);
    
    model.setPanelWidth(getWidth());
  }

}
