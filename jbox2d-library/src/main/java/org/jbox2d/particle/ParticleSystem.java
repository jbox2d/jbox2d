package org.jbox2d.particle;

import java.lang.reflect.Array;

import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.BufferUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.World;

public class ParticleSystem {
  /** All particle types that require creating pairs */
  private static final int k_pairFlags = ParticleType.b2_springParticle;
  /** All particle types that require creating triads */
  private static final int k_triadFlags = ParticleType.b2_elasticParticle;
  /** All particle types that require computing depth */
  private static final int k_noPressureFlags = ParticleType.b2_powderParticle;

  static final int xTruncBits = 12;
  static final int yTruncBits = 12;
  static final int tagBits = 8 * 4 /* sizeof(int) */;
  static final int yOffset = 1 << (yTruncBits - 1);
  static final int yShift = tagBits - yTruncBits;
  static final int xShift = tagBits - yTruncBits - xTruncBits;
  static final int xScale = 1 << xShift;
  static final int xOffset = xScale * (1 << (xTruncBits - 1));
  static final int xMask = (1 << xTruncBits) - 1;
  static final int yMask = (1 << yTruncBits) - 1;

  static int computeTag(float x, float y) {
    return ((int) (y + yOffset) << yShift) + (int) (xScale * x + xOffset);
  }

  static int computeRelativeTag(int tag, int x, int y) {
    return tag + (y << yShift) + (x << xShift);
  }

  static int limitCapacity(int capacity, int maxCount) {
    return maxCount != 0 && capacity > maxCount ? maxCount : capacity;
  }

  int m_timestamp;
  int m_allParticleFlags;
  int m_allGroupFlags;
  float m_density;
  float m_inverseDensity;
  float m_gravityScale;
  float m_particleDiameter;
  float m_inverseDiameter;
  float m_squaredDiameter;

  int m_count;
  int m_internalAllocatedCapacity;
  int m_maxCount;
  ParticleBufferInt m_flagsBuffer;
  ParticleBuffer<Vec2> m_positionBuffer;
  ParticleBuffer<Vec2> m_velocityBuffer;
  float[] m_accumulationBuffer; // temporary values
  Vec2[] m_accumulation2Buffer; // temporary vector values
  float[] m_depthBuffer; // distance from the surface

  ParticleBuffer<ParticleColor> m_colorBuffer;
  ParticleGroup[] m_groupBuffer;
  ParticleBuffer<Object> m_userDataBuffer;

  int m_proxyCount;
  int m_proxyCapacity;
  Proxy[] m_proxyBuffer;

  int m_contactCount;
  int m_contactCapacity;
  ParticleContact[] m_contactBuffer;

  int m_bodyContactCount;
  int m_bodyContactCapacity;
  ParticleBodyContact[] m_bodyContactBuffer;

  int m_pairCount;
  int m_pairCapacity;
  Pair[] m_pairBuffer;

  int m_triadCount;
  int m_triadCapacity;
  Triad[] m_triadBuffer;

  int m_groupCount;
  ParticleGroup[] m_groupList;

  float m_pressureStrength;
  float m_dampingStrength;
  float m_elasticStrength;
  float m_springStrength;
  float m_viscousStrength;
  float m_surfaceTensionStrengthA;
  float m_surfaceTensionStrengthB;
  float m_powderStrength;
  float m_ejectionStrength;
  float m_colorMixingStrength;

  World m_world;

  public ParticleSystem() {
    m_timestamp = 0;
    m_allParticleFlags = 0;
    m_allGroupFlags = 0;
    m_density = 1;
    m_inverseDensity = 1;
    m_gravityScale = 1;
    m_particleDiameter = 1;
    m_inverseDiameter = 1;
    m_squaredDiameter = 1;

    m_count = 0;
    m_internalAllocatedCapacity = 0;
    m_maxCount = 0;

    m_proxyCount = 0;
    m_proxyCapacity = 0;

    m_contactCount = 0;
    m_contactCapacity = 0;

    m_bodyContactCount = 0;
    m_bodyContactCapacity = 0;

    m_pairCount = 0;
    m_pairCapacity = 0;

    m_triadCount = 0;
    m_triadCapacity = 0;

    m_groupCount = 0;

    m_pressureStrength = 0.05f;
    m_dampingStrength = 1.0f;
    m_elasticStrength = 0.25f;
    m_springStrength = 0.25f;
    m_viscousStrength = 0.25f;
    m_surfaceTensionStrengthA = 0.1f;
    m_surfaceTensionStrengthB = 0.2f;
    m_powderStrength = 0.5f;
    m_ejectionStrength = 0.5f;
    m_colorMixingStrength = 0.5f;

    m_positionBuffer = new ParticleBuffer<Vec2>(Vec2.class);
    m_velocityBuffer = new ParticleBuffer<Vec2>(Vec2.class);
    m_colorBuffer = new ParticleBuffer<ParticleColor>(ParticleColor.class);
    m_userDataBuffer = new ParticleBuffer<Object>(Object.class);
  }

  int createParticle(ParticleDef def) {
    if (m_count >= m_internalAllocatedCapacity) {
      int capacity = m_count != 0 ? 2 * m_count : Settings.minParticleBufferCapacity;
      capacity = limitCapacity(capacity, m_maxCount);
      capacity = limitCapacity(capacity, m_flagsBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_positionBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_velocityBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_colorBuffer.userSuppliedCapacity);
      capacity = limitCapacity(capacity, m_userDataBuffer.userSuppliedCapacity);
      if (m_internalAllocatedCapacity < capacity) {
        m_flagsBuffer.data =
            reallocateBuffer(m_flagsBuffer, m_internalAllocatedCapacity, capacity, false);
        m_positionBuffer.data =
            reallocateBuffer(m_positionBuffer, m_internalAllocatedCapacity, capacity, false);
        m_velocityBuffer.data =
            reallocateBuffer(m_velocityBuffer, m_internalAllocatedCapacity, capacity, false);
        m_accumulationBuffer =
            BufferUtils.reallocateBuffer(m_accumulationBuffer, 0, m_internalAllocatedCapacity,
                capacity, false);
        m_accumulation2Buffer =
            BufferUtils.reallocateBuffer(Vec2.class, m_accumulation2Buffer, 0,
                m_internalAllocatedCapacity, capacity, true);
        m_depthBuffer =
            BufferUtils.reallocateBuffer(m_depthBuffer, 0, m_internalAllocatedCapacity, capacity,
                true);
        m_colorBuffer.data =
            reallocateBuffer(m_colorBuffer, m_internalAllocatedCapacity, capacity, true);
        m_groupBuffer =
            BufferUtils.reallocateBuffer(ParticleGroup.class, m_groupBuffer, 0,
                m_internalAllocatedCapacity, capacity, false);
        m_userDataBuffer.data =
            reallocateBuffer(m_userDataBuffer, m_internalAllocatedCapacity, capacity, true);
        m_internalAllocatedCapacity = capacity;
      }
    }
    if (m_count >= m_internalAllocatedCapacity) {
      return Settings.invalidParticleIndex;
    }
    int index = m_count++;
    m_flagsBuffer.data[index] = def.flags;
    m_positionBuffer.data[index] = def.position;
    m_velocityBuffer.data[index] = def.velocity;
    m_groupBuffer[index] = null;
    if (m_depthBuffer != null) {
      m_depthBuffer[index] = 0;
    }
    if (m_colorBuffer.data != null || !def.color.isZero()) {
      m_colorBuffer.data = requestParticleBuffer(m_colorBuffer.dataClass, m_colorBuffer.data);
      m_colorBuffer.data[index] = def.color;
    }
    if (m_userDataBuffer.data != null || def.userData != null) {
      m_userDataBuffer.data =
          requestParticleBuffer(m_userDataBuffer.dataClass, m_userDataBuffer.data);
      m_userDataBuffer.data[index] = def.userData;
    }
    if (m_proxyCount >= m_proxyCapacity) {
      int oldCapacity = m_proxyCapacity;
      int newCapacity = m_proxyCount != 0 ? 2 * m_proxyCount : Settings.minParticleBufferCapacity;
      m_proxyBuffer =
          BufferUtils.reallocateBuffer(Proxy.class, m_proxyBuffer, oldCapacity, newCapacity);
      m_proxyCapacity = newCapacity;
    }
    m_proxyBuffer[m_proxyCount++].index = index;
    return index;
  }

  void destroyParticle(int index, boolean callDestructionListener) {
    int flags = ParticleType.b2_zombieParticle;
    if (callDestructionListener) {
      flags |= ParticleType.b2_destructionListener;
    }
    m_flagsBuffer.data[index] |= flags;
  }

  private final AABB temp = new AABB();

  int destroyParticlesInShape(Shape shape, Transform xf, boolean callDestructionListener) {
    DestroyParticlesInShapeCallback callback =
        new DestroyParticlesInShapeCallback(this, shape, xf, callDestructionListener);
    shape.computeAABB(temp, xf, 0);
    m_world.queryAABB(callback, temp);  
    return callback.destroyed;
  }

  // reallocate a buffer
  static <T> T[] reallocateBuffer(ParticleBuffer<T> buffer, int oldCapacity, int newCapacity,
      boolean deferred) {
    assert (newCapacity > oldCapacity);
    return BufferUtils.reallocateBuffer(buffer.dataClass, buffer.data, buffer.userSuppliedCapacity,
        oldCapacity, newCapacity, deferred);
  }

  static int[] reallocateBuffer(ParticleBufferInt buffer, int oldCapacity, int newCapacity,
      boolean deferred) {
    assert (newCapacity > oldCapacity);
    return BufferUtils.reallocateBuffer(buffer.data, buffer.userSuppliedCapacity, oldCapacity,
        newCapacity, deferred);
  }

  @SuppressWarnings("unchecked")
  <T> T[] requestParticleBuffer(Class<T> klass, T[] buffer) {
    if (buffer == null) {
      buffer = (T[]) Array.newInstance(klass, m_internalAllocatedCapacity);
    }
    return buffer;
  }

  float[] requestParticleBuffer(float[] buffer) {
    if (buffer == null) {
      buffer = new float[m_internalAllocatedCapacity];
    }
    return buffer;
  }

  static class ParticleBuffer<T> {
    T[] data;
    final Class<T> dataClass;
    int userSuppliedCapacity;

    public ParticleBuffer(Class<T> dataClass) {
      this.dataClass = dataClass;
    }
  }
  static class ParticleBufferInt {
    int[] data;
    int userSuppliedCapacity;
  }

  /** Used for detecting particle contacts */
  static class Proxy {
    int index;
    int tag;
  }

  /** Connection between two particles */
  static class Pair {
    int indexA, indexB;
    int flags;
    float strength;
    float distance;
  }

  /** Connection between three particles */
  static class Triad {
    int indexA, indexB, indexC;
    int flags;
    float strength;
    final Vec2 pa = new Vec2(), pb = new Vec2(), pc = new Vec2();
    float ka, kb, kc, s;
  }

  // Callback used with b2VoronoiDiagram.
  static class CreateParticleGroupCallback {
    void callback(int a, int b, int c) {

    }

    ParticleSystem system;
    ParticleGroupDef def; // pointer
    int firstIndex;
  }

  // Callback used with b2VoronoiDiagram.
  static class JoinParticleGroupsCallback {
    void callback(int a, int b, int c) {

    }

    ParticleSystem system;
    ParticleGroup groupA;
    ParticleGroup groupB;
  };

  static class DestroyParticlesInShapeCallback implements QueryCallback {
    final ParticleSystem system;
    final Shape shape;
    final Transform xf;
    final boolean callDestructionListener;
    int destroyed;

    public DestroyParticlesInShapeCallback(ParticleSystem system, Shape shape, Transform xf,
        boolean callDestructionListener) {
      this.system = system;
      this.shape = shape;
      this.xf = xf;
      this.callDestructionListener = callDestructionListener;
    }

    @Override
    public boolean reportFixture(Fixture fixture) {
      return false;
    }

    public boolean reportParticle(int index) {
      assert (index >= 0 && index < system.m_count);
      if (shape.testPoint(xf, system.m_positionBuffer.data[index])) {
        system.destroyParticle(index, callDestructionListener);
        destroyed++;
      }
      return true;
    }

  }
}
