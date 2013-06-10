package org.jbox2d.collision.broadphase;

import org.jbox2d.callbacks.TreeRayCastCallback;
import org.jbox2d.collision.RayCastInput;

public class RaycastWrapper {

  private TreeRayCastCallback callback;

  private final RayCastInput subInput = new RayCastInput();

  public RaycastWrapper() {}

  public float callback(float p1x, float p1y, float p2x, float p2y, float maxFraction, int nodeId) {
    subInput.p1.x = p1x;
    subInput.p1.y = p1y;
    subInput.p2.x = p2x;
    subInput.p2.y = p2y;
    subInput.maxFraction = maxFraction;
    return callback.raycastCallback(subInput, nodeId);
  }

  public void setCallback(TreeRayCastCallback callback) {
    this.callback = callback;
  }
}
