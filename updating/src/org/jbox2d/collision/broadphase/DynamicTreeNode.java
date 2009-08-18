package org.jbox2d.collision.broadphase;

import org.jbox2d.collision.AABB;


public class DynamicTreeNode implements Comparable<DynamicTreeNode> {
	/**
	 * This is the fattened AABB
	 */
	public final AABB aabb = new AABB();
	
	public Object userData;
	
	protected DynamicTreeNode parent;
	
	protected DynamicTreeNode child1;
	protected DynamicTreeNode child2;
	
	public final boolean isLeaf(){
		return child1 == null;
	}
	
	public DynamicTreeNode(){}
	
	public int compareTo(DynamicTreeNode argNode){
		return compareCode() < argNode.compareCode() ? -1 : 1;
	}
	
	private final int compareCode(){
		int hash = 1;
		hash = 19*hash + (int)aabb.lowerBound.x;
		hash = 19*hash + (int)aabb.lowerBound.y;
		hash = 19*hash + (int)aabb.upperBound.x;
		hash = 19*hash + (int)aabb.upperBound.y;
		return hash;
	}
}
