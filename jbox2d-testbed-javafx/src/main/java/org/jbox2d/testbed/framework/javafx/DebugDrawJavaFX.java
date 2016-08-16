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
package org.jbox2d.testbed.framework.javafx;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.collision.AABB;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Mat22;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.particle.ParticleColor;
import org.jbox2d.pooling.arrays.Vec2Array;
import org.jbox2d.testbed.pooling.ColorPool;

import javafx.geometry.Rectangle2D;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.transform.Affine;

/**
 * Implementation of {@link DebugDraw} that uses JavaFX! Hooray!</br>
 * 
 * @author Daniel Murphy Initial Java 2D implementation
 * @author Hallvard Traetteberg JavaFX port
 */
public class DebugDrawJavaFX extends DebugDraw {
  public static int circlePoints = 13;
  public static final float edgeWidth = 0.02f;

  private final TestPanelJavaFX panel;
  private final ColorPool<Color> cpool = new ColorPool<Color>() {
    protected Color newColor(float r, float g, float b, float alpha) {
      return new Color(r, g, b, alpha);
    }
  };
  private final boolean yFlip;
  private Rectangle2D circle;

  public DebugDrawJavaFX(TestPanelJavaFX argTestPanel, boolean yFlip) {
    panel = argTestPanel;
    this.yFlip = yFlip;
    stroke = 1.0;
    circle = new Rectangle2D(-1, -1, 2, 2);
  }

  @Override
  public void setViewportTransform(IViewportTransform viewportTransform) {
    super.setViewportTransform(viewportTransform);
    viewportTransform.setYFlip(yFlip);
  }

  private final Vec2Array vec2Array = new Vec2Array();

  @Override
  public void drawPoint(Vec2 argPoint, float argRadiusOnScreen, Color3f argColor) {
    getWorldToScreenToOut(argPoint, sp1);
    GraphicsContext g = getGraphics();

    Color c = cpool.getColor(argColor.x, argColor.y, argColor.z);
    g.setStroke(c);
    sp1.x -= argRadiusOnScreen;
    sp1.y -= argRadiusOnScreen;
    g.fillOval((int) sp1.x, (int) sp1.y, (int) argRadiusOnScreen * 2, (int) argRadiusOnScreen * 2);
  }

  private final Vec2 sp1 = new Vec2();
  private final Vec2 sp2 = new Vec2();

  @Override
  public void drawSegment(Vec2 p1, Vec2 p2, Color3f color) {
    getWorldToScreenToOut(p1, sp1);
    getWorldToScreenToOut(p2, sp2);

    Color c = cpool.getColor(color.x, color.y, color.z);
    GraphicsContext g = getGraphics();
    g.setStroke(c);
    g.setLineWidth(0.0);
    g.strokeLine(sp1.x, sp1.y, sp2.x, sp2.y);
  }

  public void drawAABB(AABB argAABB, Color3f color) {
    Vec2 vecs[] = vec2Array.get(4);
    argAABB.getVertices(vecs);
    drawPolygon(vecs, 4, color);
  }

  private Affine tr = new Affine(), oldTrans = new Affine();
  private double stroke, oldStroke;

  private void saveState(GraphicsContext g) {
    oldTrans = g.getTransform();
    oldStroke = g.getLineWidth();
  }

  private void restoreState(GraphicsContext g) {
    g.setTransform(oldTrans);
    g.setLineWidth(oldStroke);
  }

  private double transformGraphics(GraphicsContext g, Vec2 center) {
    Vec2 e = viewportTransform.getExtents();
    Vec2 vc = viewportTransform.getCenter();
    Mat22 vt = viewportTransform.getMat22Representation();

    int flip = yFlip ? -1 : 1;
    tr.setToTransform(vt.ex.x, flip * vt.ex.y, e.x, vt.ey.x, flip * vt.ey.y, e.y);
    tr.appendTranslation(-vc.x, -vc.y);
    tr.appendTranslation(center.x, center.y);
    g.setTransform(tr);
    return vt.ex.x;
  }

  @Override
  public void drawCircle(Vec2 center, float radius, Color3f color) {
    GraphicsContext g = getGraphics();
    Color s = cpool.getColor(color.x, color.y, color.z, 1.0f);
    saveState(g);
    double scaling = transformGraphics(g, center) * radius;
    g.setLineWidth(stroke / scaling);
    g.scale(radius, radius);
    g.setStroke(s);
    g.strokeOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
    restoreState(g);
  }

  @Override
  public void drawCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    GraphicsContext g = getGraphics();
    Color s = cpool.getColor(color.x, color.y, color.z, 1f);
    saveState(g);
    double scaling = transformGraphics(g, center) * radius;
    g.setLineWidth(stroke / scaling);
    g.scale(radius, radius);
    g.setStroke(s);
    g.strokeOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
    if (axis != null) {
      g.rotate(MathUtils.atan2(axis.y, axis.x));
      g.strokeLine(0, 0, 1, 0);
    }
    restoreState(g);
  }

  @Override
  public void drawSolidCircle(Vec2 center, float radius, Vec2 axis, Color3f color) {
    GraphicsContext g = getGraphics();
    Color f = cpool.getColor(color.x, color.y, color.z, .4f);
    Color s = cpool.getColor(color.x, color.y, color.z, 1f);
    saveState(g);
    double scaling = transformGraphics(g, center) * radius;
    g.setLineWidth(stroke / scaling);
    g.scale(radius, radius);
    g.setFill(f);
    g.fillOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
    g.setStroke(s);
    g.strokeOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
    if (axis != null) {
      g.rotate(MathUtils.atan2(axis.y, axis.x));
      g.strokeLine(0, 0, 1, 0);
    }
    restoreState(g);
  }

  private final Vec2 zero = new Vec2();
  private final Color pcolorA = new Color(1f, 1f, 1f, .4f);

  @Override
  public void drawParticles(Vec2[] centers, float radius, ParticleColor[] colors, int count) {
    GraphicsContext g = getGraphics();
    saveState(g);
    double scaling = transformGraphics(g, zero) * radius;
    g.setLineWidth(stroke / scaling);
    for (int i = 0; i < count; i++) {
      Vec2 center = centers[i];
      Color color;
      if (colors == null) {
        color = pcolorA;
      } else {
        ParticleColor c = colors[i];
        color = cpool.getColor(c.r * 1f / 127, c.g * 1f / 127, c.b * 1f / 127, c.a * 1f / 127);
      }
      Affine old = g.getTransform();
      g.translate(center.x, center.y);
      g.scale(radius, radius);
      g.setFill(color);
      g.fillOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
      g.setTransform(old);
    }
    restoreState(g);
  }

  private final Color pcolor = new Color(1f, 1f, 1f, 1f);

  @Override
  public void drawParticlesWireframe(Vec2[] centers, float radius, ParticleColor[] colors,
      int count) {
    GraphicsContext g = getGraphics();
    saveState(g);
    double scaling = transformGraphics(g, zero) * radius;
    g.setLineWidth(stroke / scaling);
    for (int i = 0; i < count; i++) {
      Vec2 center = centers[i];
      Color color;
      // No alpha channel, it slows everything down way too much.
      if (colors == null) {
        color = pcolor;
      } else {
        ParticleColor c = colors[i];
        color = new Color(c.r * 1f / 127, c.g * 1f / 127, c.b * 1f / 127, 1);
      }
      Affine old = g.getTransform();
      g.translate(center.x, center.y);
      g.scale(radius, radius);
      g.setStroke(color);
      g.strokeOval(circle.getMinX(), circle.getMinX(), circle.getWidth(), circle.getHeight());
      g.setTransform(old);
    }
    restoreState(g);
  }

  private final Vec2 temp = new Vec2();
  private final static DoubleArray xDoublePool = new DoubleArray();
  private final static DoubleArray yDoublePool = new DoubleArray();

  @Override
  public void drawSolidPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    Color f = cpool.getColor(color.x, color.y, color.z, .4f);
    Color s = cpool.getColor(color.x, color.y, color.z, 1f);
    GraphicsContext g = getGraphics();
    saveState(g);
    double[] xs = xDoublePool.get(vertexCount);
    double[] ys = yDoublePool.get(vertexCount);
    for (int i = 0; i < vertexCount; i++) {
      getWorldToScreenToOut(vertices[i], temp);
      xs[i] = temp.x;
      ys[i] = temp.y;
    }
    g.setLineWidth(stroke);
    g.setFill(f);
    g.fillPolygon(xs, ys, vertexCount);
    g.setStroke(s);
    g.strokePolygon(xs, ys, vertexCount);
    restoreState(g);
  }

  @Override
  public void drawPolygon(Vec2[] vertices, int vertexCount, Color3f color) {
    Color s = cpool.getColor(color.x, color.y, color.z, 1f);
    GraphicsContext g = getGraphics();
    saveState(g);
    double[] xs = xDoublePool.get(vertexCount);
    double[] ys = yDoublePool.get(vertexCount);
    for (int i = 0; i < vertexCount; i++) {
      getWorldToScreenToOut(vertices[i], temp);
      xs[i] = temp.x;
      ys[i] = temp.y;
    }
    g.setLineWidth(stroke);
    g.setStroke(s);
    g.strokePolygon(xs, ys, vertexCount);
    restoreState(g);
  }

  @Override
  public void drawString(float x, float y, String s, Color3f color) {
    GraphicsContext g = getGraphics();
    if (g == null) {
      return;
    }
    Color c = cpool.getColor(color.x, color.y, color.z);
    g.setFill(c);
    g.fillText(s, x, y);
  }

  private GraphicsContext getGraphics() {
    return panel.getDBGraphics();
  }

  private final Vec2 temp2 = new Vec2();

  @Override
  public void drawTransform(Transform xf) {
    GraphicsContext g = getGraphics();
    getWorldToScreenToOut(xf.p, temp);
    temp2.setZero();
    float k_axisScale = 0.4f;

    Color c = cpool.getColor(1, 0, 0);
    g.setStroke(c);

    temp2.x = xf.p.x + k_axisScale * xf.q.c;
    temp2.y = xf.p.y + k_axisScale * xf.q.s;
    getWorldToScreenToOut(temp2, temp2);
    g.strokeLine(temp.x, temp.y, temp2.x, temp2.y);

    c = cpool.getColor(0, 1, 0);
    g.setStroke(c);
    temp2.x = xf.p.x + -k_axisScale * xf.q.s;
    temp2.y = xf.p.y + k_axisScale * xf.q.c;
    getWorldToScreenToOut(temp2, temp2);
    g.strokeLine(temp.x, temp.y, temp2.x, temp2.y);
  }
}
