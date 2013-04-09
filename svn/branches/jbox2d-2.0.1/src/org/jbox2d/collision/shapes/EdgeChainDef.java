package org.jbox2d.collision.shapes;

import java.util.ArrayList;
import java.util.List;

import org.jbox2d.common.Vec2;

public class EdgeChainDef extends ShapeDef {

	/** The vertices in local coordinates. */
	private final List<Vec2> vertices;

	/** Whether to create an extra edge between the first and last vertices. */
	private boolean isALoop;

	public EdgeChainDef() {
		type = ShapeType.EDGE_SHAPE;
		isALoop = true;
		vertices = new ArrayList<Vec2>();
	}

	/**
	 * Add a vertex to the chain.
	 * @param newV
	 */
	public void addVertex(final Vec2 newV) {
		vertices.add(newV);
	}

	/**
	 * Get the number of vertices in the chain.
	 * @return
	 */
	public int getVertexCount() {
		return vertices.size();
	}

	/**
	 * Is the chain a closed loop?  If so,
	 * an extra edge will be created between the
	 * first and last vertices.
	 */
	public boolean isLoop() {
		return isALoop;
	}

	/**
	 * Set whether an extra edge should be
	 * created between first and last vertices.
	 * @param isLoop True if the chain should be a closed loop
	 */
	public void setIsLoop(final boolean isLoop) {
		isALoop = isLoop;
	}

	/**
	 * Return the raw vertex list.  Modifications
	 * will effect the edge chain.
	 * @return
	 */
	public List<Vec2> getVertices() {
		return vertices;
	}

}
