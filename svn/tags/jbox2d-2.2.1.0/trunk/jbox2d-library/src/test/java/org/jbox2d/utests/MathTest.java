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
/**
 * Created at 4:32:38 AM Jan 14, 2011
 */
package org.jbox2d.utests;

import java.util.Random;

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Mat33;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Vec2;
import org.jbox2d.common.Vec3;

import junit.framework.TestCase;

/**
 * @author Daniel Murphy
 */
public class MathTest extends TestCase {

  private final static int MAX = (int) (Float.MAX_VALUE / 1000);
  private final static int RAND_ITERS = 100;

  public void testFastMath() {
    Random r = new Random();
    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      assertEquals((int) Math.floor(a), MathUtils.floor(a));
    }

    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      assertEquals((int) Math.ceil(a), MathUtils.ceil(a));
    }

    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      float b = r.nextFloat() * MAX - MAX / 2;
      assertEquals(Math.max(a, b), MathUtils.max(a, b));
    }

    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      float b = r.nextFloat() * MAX - MAX / 2;
      assertEquals(Math.min(a, b), MathUtils.min(a, b));
    }

    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      assertEquals(Math.round(a), MathUtils.round(a));
    }

    for (int i = 0; i < RAND_ITERS; i++) {
      float a = r.nextFloat() * MAX - MAX / 2;
      assertEquals(Math.abs(a), MathUtils.abs(a));
    }
  }

  public void testVec2() {
    Vec2 v = new Vec2();
    v.x = 0;
    v.y = 1;
    v.subLocal(new Vec2(10, 10));
    assertEquals(-10f, v.x);
    assertEquals(-9f, v.y);

    Vec2 v2 = v.add(new Vec2(1, 1));
    assertEquals(-9f, v2.x);
    assertEquals(-8f, v2.y);
    assertFalse(v.equals(v2));
  }

  public void testMat22Unsafes() {
    Vec2 v1 = new Vec2(10, -1.3f);
    Mat22 m1 = new Mat22(1, 34, -3, 3);
    Mat22 m2 = new Mat22(2, -1, 4.1f, -4);
    Vec2 vo = new Vec2();
    Mat22 mo = new Mat22();

    Mat22.mulToOutUnsafe(m1, m2, mo);
    assertEquals(Mat22.mul(m1, m2), mo);

    Mat22.mulToOutUnsafe(m1, v1, vo);
    assertEquals(Mat22.mul(m1, v1), vo);

    Mat22.mulTransToOutUnsafe(m1, m2, mo);
    assertEquals(Mat22.mulTrans(m1, m2), mo);

    Mat22.mulTransToOutUnsafe(m1, v1, vo);
    assertEquals(Mat22.mulTrans(m1, v1), vo);
  }

  public void testMat33() {
    Mat33 mat = new Mat33();

    mat.ex.set(3, 19, -5);
    mat.ey.set(-1, 1, 4);
    mat.ez.set(-10, 4, 4);

    Vec3 b = new Vec3(4, 1, 2);
    assertEquals(new Vec3(0.096f, 1.1013334f, -.48133332f), mat.solve33(b));

    Vec2 b2 = new Vec2(4, 1);
    assertEquals(new Vec2(0.22727273f, -3.318182f), mat.solve22(b2));
  }

  public void testVec3() {
    Vec3 v1 = new Vec3();
    Vec3 v2 = new Vec3();

    assertEquals(new Vec3(1, -15, 36), Vec3.cross(v1.set(9, 3, 1), v2.set(3, 5, 2)));
  }
}
