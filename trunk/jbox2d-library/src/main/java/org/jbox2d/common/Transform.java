/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

package org.jbox2d.common;

import java.io.Serializable;

// updated to rev 100

/**
 * A transform contains translation and rotation. It is used to represent the position and
 * orientation of rigid frames.
 */
public class Transform implements Serializable {
  private static final long serialVersionUID = 1L;

  /** The translation caused by the transform */
  public final Vec2 p;

  /** A matrix representing a rotation */
  public final Mat22 q;

  /** The default constructor. */
  public Transform() {
    p = new Vec2();
    q = new Mat22();
  }

  /** Initialize as a copy of another transform. */
  public Transform(final Transform xf) {
    p = xf.p.clone();
    q = xf.q.clone();
  }

  /** Initialize using a position vector and a rotation matrix. */
  public Transform(final Vec2 _position, final Mat22 _R) {
    p = _position.clone();
    q = _R.clone();
  }

  /** Set this to equal another transform. */
  public final Transform set(final Transform xf) {
    p.set(xf.p);
    q.set(xf.q);
    return this;
  }

  /**
   * Set this based on the position and angle.
   * 
   * @param p
   * @param angle
   */
  public final void set(Vec2 p, float angle) {
    this.p.set(p);
    q.set(angle);
  }

  /**
   * Calculate the angle that the rotation matrix represents.
   */
  public final float getAngle() {
    return MathUtils.atan2(q.ex.y, q.ex.x);
  }

  /** Set this to the identity transform. */
  public final void setIdentity() {
    p.setZero();
    q.setIdentity();
  }

  public final static Vec2 mul(final Transform T, final Vec2 v) {
    return new Vec2(T.p.x + T.q.ex.x * v.x + T.q.ey.x * v.y, T.p.y + T.q.ex.y * v.x + T.q.ey.y
        * v.y);
  }

  public final static void mulToOut(final Transform T, final Vec2 v, final Vec2 out) {
    final float tempy = T.p.y + T.q.ex.y * v.x + T.q.ey.y * v.y;
    out.x = T.p.x + T.q.ex.x * v.x + T.q.ey.x * v.y;
    out.y = tempy;
  }

  public final static void mulToOutUnsafe(final Transform T, final Vec2 v, final Vec2 out) {
    assert(v != out);
    out.x = T.p.x + T.q.ex.x * v.x + T.q.ey.x * v.y;
    out.y = T.p.y + T.q.ex.y * v.x + T.q.ey.y * v.y;
  }

  public final static Vec2 mulTrans(final Transform T, final Vec2 v) {
    final float v1x = v.x - T.p.x;
    final float v1y = v.y - T.p.y;
    final Vec2 b = T.q.ex;
    final Vec2 b1 = T.q.ey;
    return new Vec2((v1x * b.x + v1y * b.y), (v1x * b1.x + v1y * b1.y));
    // return T.R.mulT(v.sub(T.position));
  }

  public final static void mulTransToOut(final Transform T, final Vec2 v, final Vec2 out) {
    final float v1x = v.x - T.p.x;
    final float v1y = v.y - T.p.y;
    final Vec2 b = T.q.ex;
    final Vec2 b1 = T.q.ey;
    final float tempy = v1x * b1.x + v1y * b1.y;
    out.x = v1x * b.x + v1y * b.y;
    out.y = tempy;
  }

  @Override
  public final String toString() {
    String s = "XForm:\n";
    s += "Position: " + p + "\n";
    s += "R: \n" + q + "\n";
    return s;
  }
}
