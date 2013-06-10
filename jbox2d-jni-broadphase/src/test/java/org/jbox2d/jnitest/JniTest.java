package org.jbox2d.jnitest;

import org.jbox2d.collision.broadphase.DynamicTreeJNI;
import org.junit.Test;


public class JniTest {

  @Test
  public void testClass() {
    
    DynamicTreeJNI jni = new DynamicTreeJNI();
    jni.getHeight();
  }
}
