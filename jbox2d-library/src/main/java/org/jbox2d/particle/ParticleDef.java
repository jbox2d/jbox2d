package org.jbox2d.particle;

import org.jbox2d.common.Vec2;

public class ParticleDef {
  /**
   * Specifies the type of particle. A particle may be more than one type. Multiple types are
   * chained by logical sums, for example: pd.flags = ParticleType.ELASTIC.flag |
   * ParticleType.VISCOUS.flag.
   */
  int flags;

  /** The world position of the particle. */
  public final Vec2 position = new Vec2();

  /** The linear velocity of the particle in world co-ordinates. */
  public final Vec2 velocity = new Vec2();

  /** The color of the particle. */
  public final ParticleColor color = new ParticleColor();

  /** Use this to store application-specific body data. */
  public Object userData;
}
