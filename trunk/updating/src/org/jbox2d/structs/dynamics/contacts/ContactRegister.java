package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.structs.collision.ShapeType;

public class ContactRegister {
	public ShapeType s1, s2;

    public ContactCreateFcn createFcn;

    public boolean primary;
}
