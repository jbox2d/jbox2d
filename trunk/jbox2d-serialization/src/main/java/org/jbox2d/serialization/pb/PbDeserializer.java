package org.jbox2d.serialization.pb;

import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

import org.box2d.proto.Box2D.PbFixture;
import org.box2d.proto.Box2D.PbShape;
import org.box2d.proto.Box2D.PbVec2;
import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.serialization.JbDeserializer;

public class PbDeserializer implements JbDeserializer{

	private ObjectListener listener = null;
	
	@Override
	public void setObjectListener(ObjectListener argListener) {
		listener = argListener;
	}
	
	@Override
	public World deserializeWorld(InputStream argInput) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Body deserializeBody(World argWorld, InputStream argInput) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Fixture deserializeFixture(Body argBody, InputStream argInput) throws IOException {
		PbFixture fixture = PbFixture.parseFrom(argInput);
		return deserializeFixture(argBody, fixture);
	}
	
	public Fixture deserializeFixture(Body argBody, PbFixture argFixture){
		PbFixture f = argFixture;
		
		FixtureDef fd = new FixtureDef();
		fd.density = f.getDensity();
		fd.filter.categoryBits = f.getFilter().getCategoryBits();
		fd.filter.groupIndex = f.getFilter().getGroupIndex();
		fd.filter.maskBits = f.getFilter().getMaskBits();
		fd.friction = f.getFriction();
		fd.isSensor = f.getSensor();
		fd.restitution = f.getRestitution();
		fd.shape = deserializeShape(f.getShape());
		
		Fixture fixture = argBody.createFixture(fd);
		if(listener != null){
			listener.processFixture(fixture, f.getId());
		}
		return fixture;
	}

	@Override
	public Shape deserializeShape(InputStream argInput) throws IOException {
		PbShape s = PbShape.parseFrom(argInput);
		return deserializeShape(s);
	}
	
	public Shape deserializeShape(PbShape argShape){
		PbShape s = argShape;
		
		Shape shape = null;
		switch(s.getType()){
			case CIRCLE:
				CircleShape c = new CircleShape();
				c.m_p.set(pbToVec(s.getCenter()));
				c.m_radius = s.getRadius();
				shape = c;
				break;
			case POLYGON:
				PolygonShape p = new PolygonShape();
				p.m_centroid.set(pbToVec(s.getCentroid()));
				p.m_radius = s.getRadius();
				p.m_vertexCount = s.getPointsCount();
				for(int i=0; i<p.m_vertexCount; i++){
					p.m_vertices[i].set(pbToVec(s.getPoints(i)));
					p.m_normals[i].set(pbToVec(s.getNormals(i)));
				}
				shape = p;
				break;
			case EDGE:
			case LOOP:
				throw new UnsupportedOperationException("Edge and loop shapes are not supported yet.");
			default:
				throw new IllegalArgumentException("Unknown shape type");
		}
		return shape;
	}

	@Override
	public Joint deserializeJoint(World argWorld, InputStream argInput,
			Map<Integer, Body> argBodyMap, Map<Integer, Joint> argJointMap) {
		// TODO Auto-generated method stub
		return null;
	}
	
	private Vec2 pbToVec(PbVec2 argVec){
		return new Vec2(argVec.getX(), argVec.getY());
	}

}
