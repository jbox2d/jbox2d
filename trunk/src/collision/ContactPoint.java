package collision;

import common.Vec2;

public class ContactPoint {
	public Vec2 position;
	public float separation;
	public float normalImpulse;
	public float tangentImpulse;
	public ContactID id;
	
	public ContactPoint(){
		position = new Vec2();
		separation = normalImpulse = tangentImpulse = 0f;
		id = new ContactID();
	}
	
}
