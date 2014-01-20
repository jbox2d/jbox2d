package org.jbox2d.particle;

/**
 * The particle type. Can be combined with | operator. Zero means liquid.
 * 
 * @author dmurph
 */
public enum ParticleType {
  WATER(0),
  /** removed after next step */
  ZOMBIE(1 << 1),
  /** zero velocity */
  WALL(1 << 2),
  /** with restitution from stretching */
  SPRING(1 << 3),
  /** with restitution from deformation */
  ELASTIC(1 << 4),
  /** with viscosity */
  VISCOUS(1 << 5),
  /** without isotropic pressure */
  POWDER(1 << 6),
  /** with surface tension */
  TENSILE(1 << 7),
  /** mixing color between contacting particles */
  COLOR_MIXING(1 << 8),
  /** call b2DestructionListener on destruction */
  DESTRUCTION_LISTENER(1 << 9);

  public final int flag;

  private ParticleType(int flag) {
    this.flag = flag;
  }
}
