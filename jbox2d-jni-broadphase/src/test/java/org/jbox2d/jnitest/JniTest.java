package org.jbox2d.jnitest;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import org.jbox2d.callbacks.PairCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.broadphase.BroadPhaseJNI;
import org.junit.Test;


public class JniTest {

  @Test
  public void testClass() {
    BroadPhaseJNI tree = new BroadPhaseJNI();

    Integer a = new Integer(193);
    Integer b = new Integer(1);
    AABB aabb = new AABB();
    aabb.lowerBound.x = -10;
    aabb.lowerBound.y = -10;

    int proxy1 = tree.createProxy(aabb, a);
    System.out.println("proxy1: " + proxy1);
    assertNotNull(tree.getFatAABB(proxy1));
    System.out.println("fat aabb: " + tree.getFatAABB(proxy1));
    assertTrue(tree.getFatAABB(proxy1).contains(aabb));

    System.out.println("getting   user data");
    assertNotNull(tree.getUserData(proxy1));
    System.out.println("verifying same user data");
    assertSame(a, tree.getUserData(proxy1));


    aabb.upperBound.x = -2;
    aabb.upperBound.y = -2;


    int proxy2 = tree.createProxy(aabb, b);
    assertNotNull(tree.getFatAABB(proxy2));
    assertTrue(tree.getFatAABB(proxy2).contains(aabb));
    assertNotNull(tree.getUserData(proxy2));
    assertSame(b, tree.getUserData(proxy2));
    
    tree.updatePairs(new PairCallback() {
      
      @Override
      public void addPair(Object userDataA, Object userDataB) {
        
      }
    });
  }
}
