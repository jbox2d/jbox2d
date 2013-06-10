package org.jbox2d.collision.broadphase;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.TreeCallback;
import org.jbox2d.callbacks.TreeRayCastCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.RayCastInput;
import org.jbox2d.common.Vec2;

public class DynamicTreeJNI implements BroadPhaseStrategy {

  @Override
  public native int createProxy(AABB aabb, Object userData);

  @Override
  public native void destroyProxy(int proxyId);

  @Override
  public native boolean moveProxy(int proxyId, AABB aabb, Vec2 displacement);

  @Override
  public native Object getUserData(int proxyId);

  @Override
  public native AABB getFatAABB(int proxyId);

  @Override
  public native void query(TreeCallback callback, AABB aabb);

  @Override
  public native void raycast(TreeRayCastCallback callback, RayCastInput input);

  @Override
  public native int computeHeight();

  @Override
  public native int getHeight();

  @Override
  public native int getMaxBalance();

  @Override
  public native float getAreaRatio();

  @Override
  public native int getInsertionCount();

  @Override
  public void drawTree(DebugDraw draw) {}
  
  public static final DynamicTreeJNI construct() {
    System.loadLibrary("DynamicTreeJNI");
    return new DynamicTreeJNI();
  }
}
