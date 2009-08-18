package org.jbox2d.structs.collision.broadphase;

import org.jbox2d.collision.broadphase.DynamicTreeNode;

public class Pair implements Comparable<Pair>{
	public DynamicTreeNode proxyA;
	public DynamicTreeNode proxyB;

	
	public final int compareTo( Pair pair){
		return compareCode() < pair.compareCode() ? -1 : 1;
	}
	
	private final int compareCode(){
		int hash = 1;
		hash = 19*hash + (int)proxyA.aabb.lowerBound.x;
		hash = 19*hash + (int)proxyA.aabb.lowerBound.y;
		hash = 19*hash + (int)proxyA.aabb.upperBound.x;
		hash = 19*hash + (int)proxyA.aabb.upperBound.y;
		hash = 19*hash + (int)proxyB.aabb.lowerBound.x;
		hash = 19*hash + (int)proxyB.aabb.lowerBound.y;
		hash = 19*hash + (int)proxyB.aabb.upperBound.x;
		hash = 19*hash + (int)proxyB.aabb.upperBound.y;
		return hash;
	}
}
