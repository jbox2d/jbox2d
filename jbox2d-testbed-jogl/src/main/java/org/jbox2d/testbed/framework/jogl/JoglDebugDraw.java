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

import java.awt.Font;

import javax.media.opengl.GL2;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.particle.ParticleColor;

import com.jogamp.opengl.util.awt.TextRenderer;

public class JoglDebugDraw extends DebugDraw {

  private final JoglPanel panel;
  private final TextRenderer text;
  private static final int NUM_CIRCLE_POINTS = 13;


  private final float[] mat = new float[16];

  public JoglDebugDraw(JoglPanel panel) {
    this.panel = panel;
    text = new TextRenderer(new Font("Courier New", Font.PLAIN, 12));

    mat[8] = 0;
    mat[9] = 0;
    mat[2] = 0;
    mat[6] = 0;
    mat[10] = 1;
    mat[14] = 0;
    mat[3] = 0;
    mat[7] = 0;
    mat[11] = 0;
    mat[15] = 1;
  }

  @Override
  public void setViewportTransform(IViewportTransform viewportTransform) {
    viewportTransform.setYFlip(false);
    super.setViewportTransform(viewportTransform);
  }


  public void transformViewport(GL2 gl, Vec2 center) {
    Vec2 e = viewportTransform.getExtents();
    Vec2 vc = viewportTransform.getCenter();
    Mat22 vt = viewportTransform.getMat22Representation();

    int f = viewportTransform.isYFlip() ? -1 : 1;
    mat[0] = vt.ex.x;
    mat[4] = vt.ey.x;
    // mat[8] = 0;
    mat[12] = e.x;
    mat[1] = f * vt.ex.y;
    mat[5] = f * vt.ey.y;
    // mat[9] = 0;
    mat[13] = e.y;
    // mat[2] = 0;
    // mat[6] = 0;
    // mat[10] = 1;
    // mat[14] = 0;
    // mat[3] = 0;
    // mat[7] = 0;
    // mat[11] = 0;
    // mat[15] = 1;

    gl.glMultMatrixf(mat, 0);
    gl.glTranslatef(center.x - vc.x, center.y - vc.y, 0);
  }

  @Override
  public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor) {
    Vec2 vec = getWorldToScreen(argPoint);
    GL2 gl = panel.getGL().getGL2();
    gl.glPointSize(argRadiusOnScreen);
    gl.glBegin(GL2.GL_POINTS);
    gl.glVertex2f(vec.x, vec.y);
    gl.glEnd();
  }

  private final Vec2 zero = new Vec2();

  @Override
  public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1f);
    for (int i = 0; i < vertexCount; i++) {
      Vec2 v = vertices[i];
      gl.glVertex2f(v.x, v.y);
    }
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    gl.glBegin(GL2.GL_TRIANGLE_FAN);
    gl.glColor4f(color.x, color.y, color.z, .4f);
    for (int i = 0; i < vertexCount; i++) {
      Vec2 v = vertices[i];
      gl.glVertex2f(v.x, v.y);
    }
    gl.glEnd();

    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1f);
    for (int i = 0; i < vertexCount; i++) {
      Vec2 v = vertices[i];
      gl.glVertex2f(v.x, v.y);
    }
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawCircle(Vec2 center, float radius, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    float theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS;
    float c = MathUtils.cos(theta);
    float s = MathUtils.sin(theta);
    float x = radius;
    float y = 0;
    float cx = center.x;
    float cy = center.y;
    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1);
    for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
      gl.glVertex3f(x + cx, y + cy, 0);
      // apply the rotation matrix
      float temp = x;
      x = c * x - s * y;
      y = s * temp + c * y;
    }
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    float theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS;
    float c = MathUtils.cos(theta);
    float s = MathUtils.sin(theta);
    float x = radius;
    float y = 0;
    float cx = center.x;
    float cy = center.y;
    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1);
    for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
      gl.glVertex3f(x + cx, y + cy, 0);
      // apply the rotation matrix
      float temp = x;
      x = c * x - s * y;
      y = s * temp + c * y;
    }
    gl.glEnd();
    gl.glBegin(GL2.GL_LINES);
    gl.glVertex3f(cx, cy, 0);
    gl.glVertex3f(cx + axis.x * radius, cy + axis.y * radius, 0);
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    float theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS;
    float c = MathUtils.cos(theta);
    float s = MathUtils.sin(theta);
    float x = radius;
    float y = 0;
    float cx = center.x;
    float cy = center.y;
    gl.glBegin(GL2.GL_TRIANGLE_FAN);
    gl.glColor4f(color.x, color.y, color.z, .4f);
    for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
      gl.glVertex3f(x + cx, y + cy, 0);
      // apply the rotation matrix
      float temp = x;
      x = c * x - s * y;
      y = s * temp + c * y;
    }
    gl.glEnd();
    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1);
    for (int i = 0; i < NUM_CIRCLE_POINTS; i++) {
      gl.glVertex3f(x + cx, y + cy, 0);
      // apply the rotation matrix
      float temp = x;
      x = c * x - s * y;
      y = s * temp + c * y;
    }
    gl.glEnd();
    gl.glBegin(GL2.GL_LINES);
    gl.glVertex3f(cx, cy, 0);
    gl.glVertex3f(cx + axis.x * radius, cy + axis.y * radius, 0);
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);
    gl.glBegin(GL2.GL_LINES);
    gl.glColor3f(color.x, color.y, color.z);
    gl.glVertex3f(p1.x, p1.y, 0);
    gl.glVertex3f(p2.x, p2.y, 0);
    gl.glEnd();
    gl.glPopMatrix();
  }

  @Override
  public void drawParticles(Vec2[] centers, float radius, ParticleColor[] colors, int count) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);

    float theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS;
    float c = MathUtils.cos(theta);
    float s = MathUtils.sin(theta);

    float x = radius;
    float y = 0;

    for (int i = 0; i < count; i++) {
      Vec2 center = centers[i];
      float cx = center.x;
      float cy = center.y;
      gl.glBegin(GL2.GL_TRIANGLE_FAN);
      if (colors == null) {
        gl.glColor4f(1, 1, 1, .4f);
      } else {
        ParticleColor color = colors[i];
        gl.glColor4b(color.r, color.g, color.b, color.a);
      }
      for (int j = 0; j < NUM_CIRCLE_POINTS; j++) {
        gl.glVertex3f(x + cx, y + cy, 0);
        float temp = x;
        x = c * x - s * y;
        y = s * temp + c * y;
      }
      gl.glEnd();
    }
    gl.glPopMatrix();
  }


  @Override
  public void drawParticlesWireframe(Vec2[] centers, float radius, ParticleColor[] colors, int count) {
    GL2 gl = panel.getGL().getGL2();
    gl.glPushMatrix();
    transformViewport(gl, zero);

    float theta = 2 * MathUtils.PI / NUM_CIRCLE_POINTS;
    float c = MathUtils.cos(theta);
    float s = MathUtils.sin(theta);

    float x = radius;
    float y = 0;

    for (int i = 0; i < count; i++) {
      Vec2 center = centers[i];
      float cx = center.x;
      float cy = center.y;
      gl.glBegin(GL2.GL_LINE_LOOP);
      if (colors == null) {
        gl.glColor4f(1, 1, 1, 1);
      } else {
        ParticleColor color = colors[i];
        gl.glColor4b(color.r, color.g, color.b, (byte) 127);
      }
      for (int j = 0; j < NUM_CIRCLE_POINTS; j++) {
        gl.glVertex3f(x + cx, y + cy, 0);
        float temp = x;
        x = c * x - s * y;
        y = s * temp + c * y;
      }
      gl.glEnd();
    }
    gl.glPopMatrix();
  }

  private final Vec2 temp = new Vec2();
  private final Vec2 temp2 = new Vec2();

  @Override
  public void drawTransform(Transform xf) {
    GL2 gl = panel.getGL().getGL2();
    getWorldToScreenToOut(xf.p, temp);
    temp2.setZero();
    float k_axisScale = 0.4f;

    gl.glBegin(GL2.GL_LINES);
    gl.glColor3f(1, 0, 0);

    temp2.x = xf.p.x + k_axisScale * xf.q.c;
    temp2.y = xf.p.y + k_axisScale * xf.q.s;
    getWorldToScreenToOut(temp2, temp2);
    gl.glVertex2f(temp.x, temp.y);
    gl.glVertex2f(temp2.x, temp2.y);

    gl.glColor3f(0, 1, 0);
    temp2.x = xf.p.x + -k_axisScale * xf.q.s;
    temp2.y = xf.p.y + k_axisScale * xf.q.c;
    getWorldToScreenToOut(temp2, temp2);
    gl.glVertex2f(temp.x, temp.y);
    gl.glVertex2f(temp2.x, temp2.y);
    gl.glEnd();
  }

  @Override
  public void drawString(float x, float y, String s, Color3f color) {
    text.beginRendering(panel.getWidth(), panel.getHeight());
    text.setColor(color.x, color.y, color.z, 1);
    text.draw(s, (int) x, panel.getHeight() - (int) y);
    text.endRendering();
  }
}
