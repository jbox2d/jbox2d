/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/ 
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.collision;

//Updated to rev 56 of b2Collision.h

//FIXME: In the C++ version, this class is a union of
//the key and the features, meaning not that it contains
//both separately, but that the same data can be accessed
//as either.  The key there is 32 bit, and each member of
//features is 8 bits.
//
//We need to figure out if this is a problem or not, because
//I have a feeling that as of right now, key is never being
//set anyways.  Initial examination seems to show that key is
//always zero. [hacked around for the moment]
public class ContactID {
    public int key;

    // UNION
    public Features features;

    public class Features {
        public int referenceFace;

        public int incidentEdge;

        public int incidentVertex;

        public int flip;

        public Features() {
            referenceFace = incidentEdge = incidentVertex = flip = 0;
        }

        public Features(Features f) {
            referenceFace = f.referenceFace;
            incidentEdge = f.incidentEdge;
            incidentVertex = f.incidentVertex;
            flip = f.flip;
        }
        
        public void set(Features f){
            referenceFace = f.referenceFace;
            incidentEdge = f.incidentEdge;
            incidentVertex = f.incidentVertex;
            flip = f.flip;
        }
        
        public boolean isEqual(Features f){
            return (referenceFace==f.referenceFace && 
                    incidentEdge==f.incidentEdge &&
                    incidentVertex==f.incidentVertex &&
                    flip==f.flip);
        }
        
        public String toString() {
        	String s = "Features: (" + this.flip + " ," + this.incidentEdge + " ," + this.incidentVertex + " ," + this.referenceFace + ")";
        	return s;
        }

    }
    
    public void zero() {
    	key = 0;
    	features.flip = 0;
    	features.incidentEdge = 0;
    	features.incidentVertex = 0;
    	features.referenceFace = 0;
    }

    public ContactID() {
        key = 0;
        features = new Features();
    }

    public ContactID(ContactID c) {
        key = c.key;
        features = new Features(c.features);
    }

}
