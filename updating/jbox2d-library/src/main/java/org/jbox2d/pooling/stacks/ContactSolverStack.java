package org.jbox2d.pooling.stacks;

import org.jbox2d.dynamics.contacts.ContactSolver;

public class ContactSolverStack extends DynamicTLStack<ContactSolver> {
	@Override
	protected ContactSolver newObjectInstance() {
		return new ContactSolver();
	}
	
}
