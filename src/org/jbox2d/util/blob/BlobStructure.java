package org.jbox2d.util.blob;

import java.util.ArrayList;

/**
 * <p>Class for a toroidal repeating blob structure.
 * Should be subclassed with code to initialize
 * the structure in appropriate ways, such as for
 * a hexagonal lattice or a uniform grid.</p>
 * <p>Blobs are defined within an AABB from 
 * (0,0)->(1,1) that is then repeated to fill the
 * full space after scaling (using the BlobMaker methods).  
 * The connections list keeps track of pairs of BlobPoints 
 * by index that are connected, and the connections* lists keep
 * track of connections outside the AABB (to the
 * corresponding points in the next regions).
 * </p>
 * <p>
 * Connections should only be defined once per pair.
 * </p>
 * <p>
 * This class does not allow for arbitrary repeated structures,
 * but most structures of interest will be expressible
 * as toroidally repeating in this way.
 * </p>
 */
public class BlobStructure {
	// Package access to all fields
	ArrayList<BlobPoint> points;
	ArrayList<IntIntFloatFloat> connections;
	ArrayList<IntIntFloatFloat> connectionsR;
	ArrayList<IntIntFloatFloat> connectionsDR;
	ArrayList<IntIntFloatFloat> connectionsD;
	ArrayList<IntIntFloatFloat> connectionsUR;
	
	float currentFrequency = 10.0f;
	float currentDamping = 0.9f;
	
	private void updateSprings() {
		for (IntIntFloatFloat iiff:connections) {
			iiff.c = currentFrequency;
			iiff.d = currentDamping;
		}
		for (IntIntFloatFloat iiff:connectionsR) {
			iiff.c = currentFrequency;
			iiff.d = currentDamping;
		}
		for (IntIntFloatFloat iiff:connectionsDR) {
			iiff.c = currentFrequency;
			iiff.d = currentDamping;
		}
		for (IntIntFloatFloat iiff:connectionsD) {
			iiff.c = currentFrequency;
			iiff.d = currentDamping;
		}
		for (IntIntFloatFloat iiff:connectionsUR) {
			iiff.c = currentFrequency;
			iiff.d = currentDamping;
		}
	}
	
	public void setSpringFrequency(float freq) {
		currentFrequency = freq;
		updateSprings();
	}
	
	public float getSpringFrequency() {
		return currentFrequency;
	}
	
	public void setSpringDamping(float damp) {
		currentDamping = damp;
		updateSprings();
	}
	
	public float getSpringDamping() {
		return currentDamping;
	}
	
	public BlobStructure() {
		points = new ArrayList<BlobPoint>();
		connections = new ArrayList<IntIntFloatFloat>();
		connectionsR = new ArrayList<IntIntFloatFloat>();
		connectionsDR = new ArrayList<IntIntFloatFloat>();
		connectionsD = new ArrayList<IntIntFloatFloat>();
		connectionsUR = new ArrayList<IntIntFloatFloat>();
	}
	
	
	public int addPoint(BlobPoint p) {
		if (p.position.x < 0.0f || p.position.x > 1.0f ||
			p.position.y < 0.0f || p.position.y > 1.0f) 
			throw new RuntimeException("Points must be within (0,0)->(1,1) in a BlobStructure.");
		points.add(p);
		return points.indexOf(p);
	}
	
	/** 
	 * Add a connection within two points in the fundamental domain.
	 * <BR><BR>
	 * Point indices can be obtained when points are added by storing the
	 * return value of the addPoint method.
	 * @param a The index in the connections ArrayList of the first point to be connected
	 * @param b The index in the connections ArrayList of the second point to be connected
	 */
	public void addConnection(int a, int b) {
		connections.add(new IntIntFloatFloat(a,b,currentFrequency,currentDamping));
	}
	
	/** 
	 * Add a connection between point at index a in the fundamental domain
	 * and point at index b in region r.
	 * <BR><BR>
	 * Point indices can be obtained when points are added by storing the
	 * return value of the addPoint method.
	 * @param a
	 * @param b
	 * @param r
	 */
	public void addConnection(int a, int b, Region r) {
		switch(r){
		case CENTER:
			addConnection(a,b);
			break;
		case RIGHT:
			connectionsR.add(new IntIntFloatFloat(a,b,currentFrequency,currentDamping));
			break;
		case DOWN_RIGHT:
			connectionsDR.add(new IntIntFloatFloat(a,b,currentFrequency,currentDamping));
			break;
		case DOWN:
			connectionsD.add(new IntIntFloatFloat(a,b,currentFrequency,currentDamping));
			break;
		case UP_RIGHT:
			connectionsUR.add(new IntIntFloatFloat(a,b,currentFrequency,currentDamping));
			break;
		}
	}
	
	class IntIntFloatFloat {
		public int a;
		public int b;
		public float c;
		public float d;
		
		public IntIntFloatFloat(int _a, int _b, float _c, float _d) {
			a = _a;
			b = _b;
			c = _c;
			d = _d;
		}
	}
	
	/**
	 * Determines what region in toroidal space we're referring to.
	 */
	enum Region { DOWN, RIGHT, DOWN_RIGHT, UP_RIGHT, CENTER }
}

