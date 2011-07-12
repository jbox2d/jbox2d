/**
 * Created at 8:10:30 AM Jan 14, 2011
 */
package org.jbox2d.serial;

import org.jbox2d.common.Vec2;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Text;

/**
 * @author Daniel Murphy
 */
public class MathSerials {
	
	public static Element serializeVec2(Document argDoc, String argName, Vec2 argVec){
		Element vec = argDoc.createElement(argName);
		Text t = argDoc.createTextNode(argVec.x+" "+argVec.y);
		vec.appendChild(t);
		return vec;
	}
	
	public static Vec2 deserializeVec2(Element argElement){
		Text t = (Text)argElement.getFirstChild();
		String s = t.getWholeText();
		String elms[] = s.split(" ");
		return new Vec2(Float.parseFloat(elms[0]), Float.parseFloat(elms[1]));
	}
}
