/**
 * Created at 1:22:43 AM Jul 5, 2010
 */
package org.jbox2d.structs.dynamics.contacts;

import org.jbox2d.dynamics.contacts.Contact;

/**
 * @author daniel
 */
public interface ContactDestroyFcn {
	public void contactDestroyFcn(Contact contact);
}
