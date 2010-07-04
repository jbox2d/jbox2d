package org.jbox2d.pooling;

import org.jbox2d.dynamics.contacts.ContactPoint;

public class TLContactPoint extends ThreadLocal<ContactPoint> {
	protected ContactPoint initialValue(){
		return new ContactPoint();
	}
}
