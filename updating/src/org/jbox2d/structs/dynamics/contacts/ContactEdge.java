package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.contacts.Contact;

/**
 * A contact edge is used to connect bodies and contacts together
 * in a contact graph where each body is a node and each contact
 * is an edge. A contact edge belongs to a doubly linked list
 * maintained in each attached body. Each contact has two contact
 * nodes, one for each attached body.
 *
 * @author daniel
 */
public class ContactEdge {
	
	/**
	 * provides quick access to the other body attached.
	 */
	public Body other;	
	
	/**
	 * the contact
	 */
	public Contact contact;
	
	/**
	 * the previous contact edge in the body's contact list
	 */
	public ContactEdge prev;	
	
	/**
	 * the next contact edge in the body's contact list
	 */
	public ContactEdge next;	
}
