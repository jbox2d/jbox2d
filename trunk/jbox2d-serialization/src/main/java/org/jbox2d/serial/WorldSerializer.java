/**
 * Created at 7:56:11 AM Jan 14, 2011
 */
package org.jbox2d.serial;

import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.World;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Text;

/**
 * @author Daniel Murphy
 */
public class WorldSerializer {
	
	public static final int VERSION = 1;

	public static void serialize(Document argDocument, World argWorld){
		
		Element root = argDocument.createElement("World");
		root.setAttribute("Version", VERSION+"");
		argDocument.appendChild(root);
		
		if(argWorld.getGravity().isValid() && !argWorld.getGravity().equals(new Vec2())){
			Element gravity = MathSerials.serializeVec2(argDocument, "Gravity", argWorld.getGravity());
			root.appendChild(gravity);
		}
		
	}
}
