package org.jbox2d.common;

import java.io.Serializable;

/**
 * Represents a rotation
 * 
 * @author Daniel
 */
public class Rot implements Serializable {
  private static final long serialVersionUID = 1L;

  private float s, c; // sin and cos

  public Rot() {
    setIdentity();
  }

  public Rot(float angle) {
    set(angle);
  }

  public float getSin() {
    return s;
  }

  public float getCos() {
    return c;
  }

  public Rot set(float angle) {
    s = MathUtils.sin(angle);
    c = MathUtils.cos(angle);
    return this;
  }

  public Rot set(Rot other) {
    s = other.s;
    c = other.c;
    return this;
  }

  public Rot setIdentity() {
    s = 0;
    c = 1;
    return this;
  }

  public float getAngle() {
    return MathUtils.atan2(s, c);
  }

  public void getXAxis(Vec2 xAxis) {
    xAxis.set(c, s);
  }

  public void getYAxis(Vec2 yAxis) {
    yAxis.set(-s, c);
  }

  @Override
  public Rot clone() {
    Rot copy = new Rot();
    copy.s = s;
    copy.c = c;
    return copy;
  }
  
  /**
   * Multiply two rotations
   * NOTE: out cannot be q or r
   * @param q
   * @param r
   * @param out
   */
  public static final void mul(Rot q, Rot r, Rot out) {
    assert(r != out);
    assert(q != out);
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    out.s = q.s * r.c + q.c * r.s;
    out.c = q.c * r.c - q.s * r.s;
  }
}
