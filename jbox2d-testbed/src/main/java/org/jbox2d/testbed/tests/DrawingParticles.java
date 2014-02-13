package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.particle.ParticleColor;
import org.jbox2d.particle.ParticleGroup;
import org.jbox2d.particle.ParticleGroupDef;
import org.jbox2d.particle.ParticleGroupType;
import org.jbox2d.particle.ParticleType;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;

public class DrawingParticles extends TestbedTest {

  ParticleGroup m_lastGroup;
  boolean m_drawing;
  int m_particleFlags;
  int m_groupFlags;
  ParticleColor color = new ParticleColor();

  @Override
  public void initTest(boolean deserialized) {
    {
      {
        PolygonShape shape = new PolygonShape();
        Vec2[] vertices =
            new Vec2[] {new Vec2(-40, -20), new Vec2(40, -20), new Vec2(40, 0), new Vec2(-40, 0)};
        shape.set(vertices, 4);
        getGroundBody().createFixture(shape, 0.0f);
      }

      {
        PolygonShape shape = new PolygonShape();
        Vec2[] vertices =
            new Vec2[] {new Vec2(-40, -20), new Vec2(-20, -20), new Vec2(-20, 60),
                new Vec2(-40, 60)};
        shape.set(vertices, 4);
        getGroundBody().createFixture(shape, 0.0f);
      }

      {
        PolygonShape shape = new PolygonShape();

        Vec2[] vertices =
            new Vec2[] {new Vec2(20, -20), new Vec2(40, -20), new Vec2(40, 60), new Vec2(20, 60)};
        shape.set(vertices, 4);
        getGroundBody().createFixture(shape, 0.0f);
      }

      {
        PolygonShape shape = new PolygonShape();
        Vec2[] vertices =
            new Vec2[] {new Vec2(-40, 40), new Vec2(40, 40), new Vec2(40, 60), new Vec2(-40, 60)};
        shape.set(vertices, 4);
        getGroundBody().createFixture(shape, 0.0f);
      }
    }

    m_world.setParticleRadius(0.5f);
    m_lastGroup = null;
    m_drawing = true;
    m_groupFlags = 0;
  }

  @Override
  public void step(TestbedSettings settings) {
    super.step(settings);

    addTextLine("Keys: (L) liquid, (E) elastic, (S) spring");
    addTextLine("(F) rigid, (W) wall, (V) viscous, (T) tensile");
    addTextLine("(Z) erase, (X) move");
  }

  public void keyPressed(char keyChar, int keyCode) {
    m_drawing = keyChar != 'x';
    m_particleFlags = 0;
    m_groupFlags = 0;
    color.set((byte)127, (byte)127, (byte)127, (byte)50);
    switch (keyChar) {
      case 'e':
        m_particleFlags = ParticleType.b2_elasticParticle;
        m_groupFlags = ParticleGroupType.b2_solidParticleGroup;
        break;
      case 'p':
        m_particleFlags = ParticleType.b2_powderParticle;
        break;
      case 'f':
        m_groupFlags =
            ParticleGroupType.b2_rigidParticleGroup | ParticleGroupType.b2_solidParticleGroup;
        break;
      case 's':
        m_particleFlags = ParticleType.b2_springParticle;
        m_groupFlags = ParticleGroupType.b2_solidParticleGroup;
        break;
      case 't':
        color.set((byte)0, (byte)127, (byte)0, (byte)50);
        m_particleFlags = ParticleType.b2_tensileParticle;
        break;
      case 'v':
        color.set((byte)0, (byte)0, (byte)127, (byte)50);
        m_particleFlags = ParticleType.b2_viscousParticle;
        break;
      case 'w':
        m_particleFlags = ParticleType.b2_wallParticle;
        m_groupFlags = ParticleGroupType.b2_solidParticleGroup;
        break;
      case 'z':
        m_particleFlags = ParticleType.b2_zombieParticle;
        break;
    }
  }

  Transform pxf = new Transform();
  CircleShape pshape = new CircleShape();
  ParticleGroupDef ppd = new ParticleGroupDef();

  @Override
  public void mouseDrag(Vec2 p, int button) {
    super.mouseDrag(p, button);
    if (m_drawing) {
      pshape.m_p.set(p);
      pshape.m_radius = 2.0f;
      pxf.setIdentity();
      m_world.destroyParticlesInShape(pshape, pxf);
      ppd.shape = pshape;
      ppd.color = color;
      ppd.flags = m_particleFlags;
      ppd.groupFlags = m_groupFlags;
      ParticleGroup group = m_world.createParticleGroup(ppd);
      if (m_lastGroup != null && group.getGroupFlags() == m_lastGroup.getGroupFlags()) {
        m_world.joinParticleGroups(m_lastGroup, group);
      } else {
        m_lastGroup = group;
      }
      mouseTracing = false;
    }
  }

  @Override
  public void mouseUp(Vec2 p, int button) {
    super.mouseUp(p, button);
    m_lastGroup = null;
  }

  @Override
  public void particleGroupDestroyed(ParticleGroup group) {
    super.particleGroupDestroyed(group);
    if (group == m_lastGroup) {
      m_lastGroup = null;
    }
  }

  @Override
  public String getTestName() {
    return "Drawing Particles";
  }
}
