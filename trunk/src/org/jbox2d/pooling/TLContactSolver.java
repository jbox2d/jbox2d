package org.jbox2d.pooling;

import org.jbox2d.dynamics.contacts.ContactSolver;

public class TLContactSolver extends ThreadLocal<ContactSolver> {
	protected ContactSolver initialValue(){
		return new ContactSolver();
	}
}
