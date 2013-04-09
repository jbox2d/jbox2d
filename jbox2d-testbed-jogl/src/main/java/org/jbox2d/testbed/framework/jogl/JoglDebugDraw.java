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
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.pooling.arrays.Vec2Array;

import com.jogamp.opengl.util.awt.TextRenderer;

public class JoglDebugDraw extends DebugDraw {

  private final JoglPanel panel;
  private final TextRenderer text;

  public JoglDebugDraw(JoglPanel argPanel) {
    super(new OBBViewportTransform());
    
    panel = argPanel;
    text = new TextRenderer(new Font("Courier New", Font.PLAIN, 12));
    viewportTransform.setYFlip(false);
  }

  @Override
  public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor) {
    Vec2 vec = getWorldToScreen(argPoint);
    GL2 gl = panel.getGL().getGL2();
    gl.glBegin(GL2.GL_POINT);
    gl.glPointSize(argRadiusOnScreen);
    gl.glVertex2f(vec.x, vec.y);
    gl.glEnd();
  }

  private final Vec2 trans = new Vec2();
  @Override
  public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glBegin(GL2.GL_TRIANGLE_FAN);
    gl.glColor4f(color.x, color.y, color.z, .4f);
    for(int i=0; i<vertexCount; i++){
      getWorldToScreenToOut(vertices[i], trans);
      gl.glVertex2f(trans.x, trans.y);
    }
    gl.glEnd();
    
    gl.glBegin(GL2.GL_LINE_LOOP);
    gl.glColor4f(color.x, color.y, color.z, 1f);
    for(int i=0; i<vertexCount; i++){
      getWorldToScreenToOut(vertices[i], trans);
      gl.glVertex2f(trans.x, trans.y);
    }
    gl.glEnd();
  }

  private final Vec2Array vec2Array = new Vec2Array();
  @Override
  public void drawCircle(Vec2 center, float radius, Color3f color) {
    Vec2[] vecs = vec2Array.get(20);
    generateCirle(center, radius, vecs, 20);
    drawPolygon(vecs, 20, color);
  }

  @Override
  public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    Vec2[] vecs = vec2Array.get(20);
    generateCirle(center, radius, vecs, 20);
    drawSolidPolygon(vecs, 20, color);
    drawSegment(center, vecs[0], color);
  }

  @Override
  public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
    GL2 gl = panel.getGL().getGL2();
    gl.glBegin(GL2.GL_LINES);
    gl.glColor3f(color.x, color.y, color.z);
    getWorldToScreenToOut(p1, trans);
    gl.glVertex2f(trans.x, trans.y);
    getWorldToScreenToOut(p2, trans);
    gl.glVertex2f(trans.x, trans.y);
    gl.glEnd();
  }

  @Override
  public void drawTransform(Transform xf) {
    // TODO Auto-generated method stub

  }

  @Override
  public void drawString(float x, float y, String s, Color3f color) {
    text.beginRendering(panel.getWidth(), panel.getHeight());
    text.setColor(color.x, color.y, color.z, 1);
    text.draw(s, (int)x,panel.getHeight() -  (int)y);
    text.endRendering();
  }

  // CIRCLE GENERATOR

  private void generateCirle(Vec2 argCenter, float argRadius, Vec2[] argPoints, int argNumPoints) {
    float inc = MathUtils.TWOPI / argNumPoints;

    for (int i = 0; i < argNumPoints; i++) {
      argPoints[i].x = (argCenter.x + MathUtils.cos(i * inc) * argRadius);
      argPoints[i].y = (argCenter.y + MathUtils.sin(i * inc) * argRadius);
    }
  }
}
