package org.jbox2d.callbacks;

import org.jbox2d.common.Vec2;

public interface ParticleRaycastCallback {
  /**
   * Called for each particle found in the query.
   * @param index
   * @param point
   * @param normal
   * @param fraction
   * @return
   */
  float reportParticle(int index, Vec2 point, Vec2 normal, float fraction);

}
