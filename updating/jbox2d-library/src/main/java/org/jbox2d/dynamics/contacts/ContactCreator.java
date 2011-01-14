package org.jbox2d.dynamics.contacts;

import org.jbox2d.dynamics.Fixture;

// updated to rev 100 - ec
public interface ContactCreator {

	public Contact contactCreateFcn(Fixture fixtureA, Fixture fixtureB);
	
	public void contactDestroyFcn(Contact contact);
}
