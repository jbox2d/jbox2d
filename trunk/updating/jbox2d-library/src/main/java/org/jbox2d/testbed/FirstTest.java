package org.jbox2d.testbed;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;

public class FirstTest {

	public static void main(String[] args){
		World world = new World(new Vec2(-1, -1), true);
		BodyDef bd = new BodyDef();
		bd.position.set(4,3);
		bd.type = BodyType.DYNAMIC;
		Body body = world.createBody(bd);
		
		FixtureDef fd = new FixtureDef();
		fd.density = 1;
		fd.shape = new CircleShape();
		Fixture f = body.createFixture(fd);
		
		world.Step(.2f, 10, 10);
		world.Step(.2f, 10, 10);
		world.Step(.2f, 10, 10);
		world.Step(.2f, 10, 10);
		world.Step(.2f, 10, 10);
		
		System.out.println(body.getPosition());
	}
}
