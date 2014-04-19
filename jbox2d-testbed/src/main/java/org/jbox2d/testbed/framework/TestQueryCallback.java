package org.jbox2d.testbed.framework;

import org.jbox2d.callbacks.QueryCallback;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;

public class TestQueryCallback implements QueryCallback {

  public final Vec2 point;
  public Fixture fixture;

  public TestQueryCallback() {
    point = new Vec2();
    fixture = null;
  }

  public boolean reportFixture(Fixture argFixture) {
    Body body = argFixture.getBody();
    if (body.getType() == BodyType.DYNAMIC) {
      boolean inside = argFixture.testPoint(point);
      if (inside) {
        fixture = argFixture;

        return false;
      }
    }

    return true;
  }
}

	