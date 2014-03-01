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

import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLCapabilities;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.GLProfile;
import javax.media.opengl.awt.GLJPanel;
import javax.media.opengl.glu.GLU;

import org.jbox2d.testbed.framework.TestbedController;
import org.jbox2d.testbed.framework.TestbedModel;
import org.jbox2d.testbed.framework.TestbedPanel;
import org.jbox2d.testbed.framework.j2d.AWTPanelHelper;

public class JoglPanel extends GLJPanel implements TestbedPanel, GLEventListener {
  private static final long serialVersionUID = 1L;

  public static final int SCREEN_DRAG_BUTTON = 3;

  public static final int INIT_WIDTH = 600;
  public static final int INIT_HEIGHT = 600;

  private final TestbedController controller;

  public JoglPanel(final TestbedModel model, final TestbedController controller) {
    super(new GLCapabilities(GLProfile.getDefault()));
    this.controller = controller;
    setSize(600, 600);
    setPreferredSize(new Dimension(600, 600));
    setAutoSwapBufferMode(true);
    addGLEventListener(this);
    AWTPanelHelper.addHelpAndPanelListeners(this, model, controller, SCREEN_DRAG_BUTTON);
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
    controller.updateTest();
    getGL().glFlush();
  }

  @Override
  public void dispose(GLAutoDrawable arg0) {}

  @Override
  public void init(GLAutoDrawable arg0) {
    getGL().getGL2().glLineWidth(1f);
    getGL().getGL2().glEnable(GL2.GL_BLEND);
    getGL().getGL2().glBlendFunc(GL2.GL_SRC_ALPHA, GL2.GL_ONE_MINUS_SRC_ALPHA);
  }

  @Override
  public void reshape(GLAutoDrawable arg0, int arg1, int arg2, int arg3, int arg4) {
    GL2 gl2 = arg0.getGL().getGL2();

    gl2.glMatrixMode(GL2.GL_PROJECTION);
    gl2.glLoadIdentity();

    // coordinate system origin at lower left with width and height same as the window
    GLU glu = new GLU();
    glu.gluOrtho2D(0.0f, getWidth(), 0.0f, getHeight());

    gl2.glMatrixMode(GL2.GL_MODELVIEW);
    gl2.glLoadIdentity();

    gl2.glViewport(0, 0, getWidth(), getHeight());

    controller.updateExtents(arg3 / 2, arg4 / 2);
  }
}
