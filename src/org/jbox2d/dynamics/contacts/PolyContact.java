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

package org.jbox2d.dynamics.contacts;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.collision.CollidePoly;
import org.jbox2d.collision.Collision;
import org.jbox2d.collision.ContactID;
import org.jbox2d.collision.ManifoldPoint;
import org.jbox2d.collision.Manifold;
import org.jbox2d.collision.PolygonShape;
import org.jbox2d.collision.Shape;
import org.jbox2d.collision.ShapeType;
import org.jbox2d.common.XForm;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.ContactListener;


public class PolyContact extends Contact implements ContactCreateFcn {

    Manifold m_manifold;

    public PolyContact(Shape s1, Shape s2) {
        super(s1, s2);
        assert (m_shape1.m_type == ShapeType.POLYGON_SHAPE);
        assert (m_shape2.m_type == ShapeType.POLYGON_SHAPE);

        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public PolyContact() {
        super();
        m_manifold = new Manifold();
        m_manifoldCount = 0;
    }

    public Contact clone() {
        PolyContact newC = new PolyContact(this.m_shape1, this.m_shape2);
        if (this.m_manifold != null) newC.m_manifold = new Manifold(this.m_manifold);
        newC.m_manifoldCount = this.m_manifoldCount;
        // The parent world.
        newC.m_world = this.m_world;
        
        newC.m_toi = this.m_toi;

        // World pool and list pointers.
        newC.m_prev = this.m_prev;
        newC.m_next = this.m_next;

        // Nodes for connecting bodies.
        newC.m_node1 = this.m_node1;
        newC.m_node2 = this.m_node2;

        // Combined friction
        newC.m_friction = this.m_friction;
        newC.m_restitution = this.m_restitution;

        newC.m_flags = this.m_flags;
        
        return newC;
    }

    @Override
    public List<Manifold> getManifolds() {
        // System.out.println("PolyContact.GetManifolds()");
        List<Manifold> ret = new ArrayList<Manifold>();
        if (m_manifold != null) {
            ret.add(m_manifold);
        }

        return ret;
    }

    public Contact create(Shape shape1, Shape shape2) {
        return new PolyContact(shape1, shape2);
    }
    
    public void dumpManifoldPoints() {
    	for (int i=0; i<m_manifold.pointCount; ++i) {
    		ManifoldPoint mp = m_manifold.points[i];
    		System.out.println("Manifold point dump: "+mp.normalForce+" "+mp.tangentForce);
    	}
    }

    @Override
    public void evaluate(ContactListener listener) {
    	Body b1 = m_shape1.getBody();
    	Body b2 = m_shape2.getBody();
        // Manifold m0 = m_manifold;
        Manifold m0 = new Manifold(m_manifold);
        //This next stuff might be unnecessary now
        for (int k = 0; k < m_manifold.pointCount; k++) {
            m0.points[k] = new ManifoldPoint(m_manifold.points[k]);
            m0.points[k].normalForce = m_manifold.points[k].normalForce;
            m0.points[k].tangentForce = m_manifold.points[k].tangentForce;
            m0.points[k].separation = m_manifold.points[k].separation;
            //m0.points[k].id.key = m_manifold.points[k].id.key;
            m0.points[k].id.features.set(m_manifold.points[k].id.features);
            //System.out.println(m_manifold.points[k].normalForce);
        }
        m0.pointCount = m_manifold.pointCount;

        CollidePoly.collidePoly(m_manifold, (PolygonShape) m_shape1,b1.m_xf,(PolygonShape) m_shape2, b2.m_xf);

        // Match contact ids to facilitate warm starting.
        // Watch out (Java note):
        // m_manifold.pointCount != m_manifold.points.length!!!
        boolean match[] = new boolean[] { false, false };
        if (m_manifold.pointCount > 0) {
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (int i = 0; i < m_manifold.pointCount; ++i) {//m_manifold.points.length; ++i) {
            	ManifoldPoint cp = m_manifold.points[i];
    			cp.normalForce = 0.0f;
    			cp.tangentForce = 0.0f;
    			boolean matched = false;
                ContactID id = new ContactID(cp.id);

                for (int j = 0; j < m0.pointCount; ++j) {
                    if (match[j] == true) {
                        continue;
                    }

                    ManifoldPoint cp0 = m0.points[j];
                    ContactID id0 = new ContactID(cp0.id);
                    id0.features.flip &= ~Collision.NEW_POINT;
                    
                    if (id0.features.isEqual(id.features)){
                    //if (id0.key == id.key) { //must use isEqual in Java, as id.key is simulating a union
                        match[j] = true;
                        cp.normalForce = cp0.normalForce;
    					cp.tangentForce = cp0.tangentForce;

    					// Not a new point.
    					matched = true;
                        break;
                    }
                }
                if (matched == false) {
    				cp.id.features.flip |= Collision.NEW_POINT;
    			}
            }
            m_manifoldCount = 1;
        }
        else {
            m_manifoldCount = 0;
            //m_manifold = null; //Cleaner to actually store the count...
        }
        
    	// Report removed points.
    	if ( (listener != null) && m0.pointCount > 0) {
    		ContactPoint cp = new ContactPoint();
    		cp.shape1 = m_shape1;
    		cp.shape2 = m_shape2;
    		cp.normal = m0.normal.clone();
    		for (int i = 0; i < m0.pointCount; ++i) {
    			if (match[i]) {
    				continue;
    			}

    			ManifoldPoint mp0 = m0.points[i];
    			cp.position = XForm.mul(b1.m_xf, mp0.localPoint1);
    			cp.separation = mp0.separation;
    			cp.normalForce = mp0.normalForce;
    			cp.tangentForce = mp0.tangentForce;
    			cp.id = new ContactID(mp0.id);
    			listener.remove(cp);
    		}
    	}
    }
    
}
