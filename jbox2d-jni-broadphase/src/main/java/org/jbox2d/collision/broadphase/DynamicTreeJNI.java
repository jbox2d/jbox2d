package org.jbox2d.collision.broadphase;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.callbacks.TreeCallback;
import org.jbox2d.callbacks.TreeRayCastCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.RayCastInput;
import org.jbox2d.common.Vec2;

public class DynamicTreeJNI implements BroadPhaseStrategy {

  static {
    NarSystem.loadLibrary();
  }

  private long nativeAddress;

  public DynamicTreeJNI() {
    createNativeTree();
  }

  private native void createNativeTree();

  @Override
  public int createProxy(AABB aabb, Object userData) {
    return createProxy(aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x, aabb.upperBound.y,
        userData);
  }

  private native int createProxy(float lowerX, float lowerY, float upperX, float upperY,
      Object userData);

  @Override
  public native void destroyProxy(int proxyId);

  @Override
  public boolean moveProxy(int proxyId, AABB aabb, Vec2 displacement) {
    return moveProxy(proxyId, aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x,
        aabb.upperBound.y, displacement.x, displacement.y);
  }

  private native boolean moveProxy(int proxyId, float lowerX, float lowerY, float upperX,
      float upperY, float displaceX, float displaceY);

  @Override
  public native Object getUserData(int proxyId);

  @Override
  public AABB getFatAABB(int proxyId) {
    return null;
  }

  @Override
  public void query(TreeCallback callback, AABB aabb) {
    query(callback, aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x, aabb.upperBound.y);
  }

  private native void query(TreeCallback callback, float lowerX, float lowerY, float upperX,
      float upperY);

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

  private native void freeNative();

  @Override
  protected void finalize() throws Throwable {
    super.finalize();
    freeNative();
  }

}
