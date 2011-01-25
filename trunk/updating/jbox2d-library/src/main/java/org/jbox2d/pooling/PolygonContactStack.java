/**
 * Created at 7:20:05 PM Jan 24, 2011
 */
package org.jbox2d.pooling;

import org.jbox2d.common.Settings;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.PolygonContact;

/**
 * @author Daniel Murphy
 */
public class PolygonContactStack extends MutableStack<Contact> {

	private final WorldPool pool;
	
	public PolygonContactStack(WorldPool argPool){
		pool = argPool;
		initStack(Settings.CONTACT_STACK_INIT_SIZE);
	}
	
	/**
	 * @see org.jbox2d.pooling.MutableStack#createArray(int, E[])
	 */
	@Override
	protected Contact[] createArray(int argSize, Contact[] argOld) {
		if(argOld != null){
			Contact[] sk = new PolygonContact[argSize];
			for(int i=0 ;i<argOld.length; i++){
				sk[i] = argOld[i];
			}
			for(int i=argOld.length; i< argSize; i++){
				sk[i] = new PolygonContact(pool);
			}
			return sk;
		}else{
			PolygonContact[] sk = new PolygonContact[argSize];
			for(int i=0; i< argSize; i++){
				sk[i] = new PolygonContact(pool);
			}
			return sk;
		}
	}

}
