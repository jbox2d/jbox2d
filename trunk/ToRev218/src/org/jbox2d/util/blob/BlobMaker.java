package org.jbox2d.util.blob;

import org.jbox2d.collision.AABB;
import org.jbox2d.collision.shapes.CircleDef;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.util.blob.BlobStructure.IntIntFloatFloat;

/**
 * BlobMaker offers a static API for the creation of blobs.
 */
public class BlobMaker {
	static public float pointRadius = 3.0f;
	static public float pointDensity = 1.0f;
	static public float pointFriction = 0.5f;
	
	
	/**
	 * Creates a blob in a given physics world.
	 * Does not apply any scaling or translation to the structure
	 * before filling the container with it.
	 * <BR><BR>
	 * The fill procedure aligns the structure with the upper
	 * left corner of the container AABB, then repeats the structure
	 * until the AABB is filled, testing at each point whether
	 * the container is supposed to have geometry there.
	 * @param s The BlobStructure definition
	 * @param c The BlobContainer that specifies the geometry to fill
	 * @param w The World to create the blob in
	 */
	static public void createBlob(BlobStructure s, BlobContainer c, World w) {
		createBlob(s,c,w,1.0f,1.0f);
	}
	
	/**
	 * Creates a blob in a given physics world.
	 * Applies the specified x/y scaling to the structure before fill.
	 * <BR><BR>
	 * The fill procedure aligns the structure with the upper
	 * left corner of the container AABB, applies the scaling,
	 * then repeats the structure until the AABB is filled, 
	 * testing at each point whether the container is supposed 
	 * to have geometry there.
	 * @param s The BlobStructure definition
	 * @param c The BlobContainer that specifies the geometry to fill
	 * @param w The World to create the blob in
	 * @param scaleX The world width of one repeating cell of the structure
	 * @param scaleY The world height of one repeating cell of the structure
	 */
	static public void createBlob(BlobStructure s, BlobContainer c, World w, 
								  float scaleX, float scaleY) {
		createBlob(s,c,w,scaleX,scaleY,0.0f,0.0f);
	}
	
	/**
	 * Creates a blob in a given physics world.
	 * Applies the specified x/y scaling to the structure before fill.
	 * <BR><BR>
	 * The fill procedure aligns the structure with the upper
	 * left corner of the container AABB, applies the scaling,
	 * shifts the cell by the requested translation amounts,
	 * then repeats the structure until the AABB is filled, 
	 * testing at each point whether the container is supposed 
	 * to have geometry there.
	 * @param s The BlobStructure definition
	 * @param c The BlobContainer that specifies the geometry to fill
	 * @param w The World to create the blob in
	 * @param scaleX The world width of one repeating cell of the structure
	 * @param scaleY The world height of one repeating cell of the structure
	 * @param transX The world x offset of the cells from the AABB edge
	 * @param transY The world y offset of the cells from the AABB edge
	 */
	static public void createBlob(BlobStructure s, BlobContainer c, World w, 
								  float scaleX, float scaleY, float transX, float transY) {
		/*
		 * Fill method:
		 * 
		 * After initial placement, we fill up the entire AABB with regions so that we
		 * achieve total coverage and overshoot by 1 cell in each direction.  We fill from
		 * left to right and then top to bottom in a scanline manner.  We keep 
		 * track of the width and height in terms of cells.  Points that are not within
		 * geometry are marked null, but still take up space in the ultimate point array.
		 * 
		 * We then iterate over the cells in the same order, using the known offsets in the
		 * big point array to reference the appropriate relative cells.  If the requested
		 * point to connect to is null, then we don't create the connection; otherwise, we
		 * do.
		 * 
		 * Simple, right?
		 * 
		 */
		
		AABB aabb = c.getAABB();
		while (transX > 0.0f) transX -= scaleX;
		while (transY > 0.0f) transY -= scaleY;
		float xMin = aabb.lowerBound.x + transX;
		float yMin = aabb.lowerBound.y + transY;
		//How many do we need in each direction to cover AABB?
		int nWidth = (int)Math.ceil((aabb.upperBound.x - xMin)/scaleX);
		int nHeight = (int)Math.ceil((aabb.upperBound.y - yMin)/scaleY);
		//System.out.println(nWidth+ " " +(aabb.upperBound.x - xMin)/scaleX);
		//System.out.println(aabb);
		//Add overshoot
		nWidth += 3;
		nHeight += 3;
		
		
		int nPerCell = s.points.size();
		int nPoints = nPerCell*nWidth*nHeight;
		
		System.out.println(nWidth + " " +nHeight);
		
		// Fill the bodies[] array
		Body[] bodies = new Body[nPoints];
		CircleDef cd = new CircleDef();
		cd.radius = pointRadius;
		cd.density = pointDensity;
		cd.friction = pointFriction;
		int index = 0;
		for (int j=0; j<nHeight; ++j) {
			float yStart = yMin + transY + j*scaleY;
			for (int i=0; i<nWidth; ++i) {
				float xStart = xMin + transX + i*scaleX;
				for (int k = 0; k<nPerCell; ++k) {
					Vec2 position = new Vec2(s.points.get(k).position.x+xStart,
											 s.points.get(k).position.y+yStart);
					if (!c.containsPoint(position)) {
						bodies[index++] = null;
						//System.out.println("Skipped point at "+position);
						continue;
					}
					//System.out.println("Creating pt at "+position);
					
					BodyDef bd = new BodyDef();
					bd.position = position;
					bd.fixedRotation = false; //causes problems when extremely compressed
					bd.angularDamping = 0.2f;
					bodies[index] = w.createBody(bd);
					bodies[index].createShape(cd);
					bodies[index].setMassFromShapes();
					++index;
				}
			}
		}
		
		//Handle the connections - only go up until the row/col right before
		//the last one, because the last ones overshoot and should have all
		//null objects.
		for (int j=0; j<nHeight; ++j) {
			int rowStartIndex = j*nWidth*nPerCell;
			for (int i=0; i<nWidth; ++i) {
				int boxStartIndex = rowStartIndex + i*nPerCell;
				int indexUR = -(nWidth-1)*nPerCell + boxStartIndex;
				int indexR = nPerCell + boxStartIndex;
				int indexDR = (nWidth+1)*nPerCell + boxStartIndex;
				int indexD = nWidth*nPerCell + boxStartIndex;
				for (int k=0; k < s.connections.size(); ++k) {
					IntIntFloatFloat iiff = s.connections.get(k);
					int a = iiff.a + boxStartIndex;
					int b = iiff.b + boxStartIndex;
					float freq = iiff.c;
					float damp = iiff.d;
					createConnection(bodies,a,b,freq,damp, w);
				}
				for (int k=0; k<s.connectionsR.size(); ++k) {
					IntIntFloatFloat iiff = s.connectionsR.get(k);
					int a = iiff.a + boxStartIndex;
					int b = iiff.b + indexR;
					float freq = iiff.c;
					float damp = iiff.d;
					createConnection(bodies,a,b,freq,damp, w);
				}
				for (int k=0; k<s.connectionsDR.size(); ++k) {
					IntIntFloatFloat iiff = s.connectionsDR.get(k);
					int a = iiff.a + boxStartIndex;
					int b = iiff.b + indexDR;
					float freq = iiff.c;
					float damp = iiff.d;
					createConnection(bodies,a,b,freq,damp, w);
				}
				for (int k=0; k<s.connectionsD.size(); ++k) {
					IntIntFloatFloat iiff = s.connectionsD.get(k);
					int a = iiff.a + boxStartIndex;
					int b = iiff.b + indexD;
					float freq = iiff.c;
					float damp = iiff.d;
					createConnection(bodies,a,b,freq,damp, w);
				}
				for (int k=0; k<s.connectionsUR.size(); ++k) {
					if (j==0) break; //First row has no row above it
					IntIntFloatFloat iiff = s.connectionsUR.get(k);
					int a = iiff.a + boxStartIndex;
					int b = iiff.b + indexUR;
					float freq = iiff.c;
					float damp = iiff.d;
					createConnection(bodies,a,b,freq,damp, w);
				}
			}
		}
	}

	private static Joint createConnection(Body[] bodies, int a, int b, float frequency, float damping, World w) {
		// If either body is null, return - the connection should be ignored.
		if (a >= bodies.length || b >= bodies.length ||
			bodies[a] == null || bodies[b] == null) return null;
		
		DistanceJointDef jd = new DistanceJointDef();
		jd.collideConnected = false;
		jd.dampingRatio = damping;
		jd.frequencyHz = frequency;
		jd.initialize(bodies[a],bodies[b],bodies[a].getMemberPosition(),bodies[b].getMemberPosition());
		return w.createJoint(jd);
	}
}
