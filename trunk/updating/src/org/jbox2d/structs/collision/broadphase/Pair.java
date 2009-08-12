package org.jbox2d.structs.collision.broadphase;

public class Pair implements Comparable<Pair>{
	public int proxyIdA;
	public int proxyIdB;
	public int next;
	
	public final int compareTo( Pair pair){
		if(proxyIdA < pair.proxyIdA){
			return -1;
		}
		
		if(proxyIdA == pair.proxyIdA){
			return (proxyIdB < pair.proxyIdB) ? -1 : 1;
		}
		
		return 1;
	}
}
