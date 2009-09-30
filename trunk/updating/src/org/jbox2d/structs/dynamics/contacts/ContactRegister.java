package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.contacts.Contact;

public interface ContactRegister {
	
	public Contact contactCreateFcn(Fixture fixtureA, Fixture fixtureB);
	public void contactDestroyFcn(Contact contact);
	
	public boolean isPrimary();
	public void setPrimary(boolean argPrimary);
}
