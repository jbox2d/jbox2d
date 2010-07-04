package org.jbox2d.util.blob;

/**
 * Simple non-reinforced square lattice.
 * Easy to collapse, but simple.
 */
public class SquareLatticeStructure extends BlobStructure {

	public SquareLatticeStructure() {
		super();
		BlobPoint toAdd = new BlobPoint(0.5f,0.5f);
		int center = addPoint(toAdd);
		
		addConnection(center,center,Region.UP_RIGHT);
		addConnection(center,center,Region.RIGHT);
		addConnection(center,center,Region.DOWN_RIGHT);
		addConnection(center,center,Region.DOWN);
	}

}
