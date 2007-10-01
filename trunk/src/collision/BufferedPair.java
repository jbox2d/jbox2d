package collision;

public class BufferedPair implements Comparable{
	int proxyId1;
	int proxyId2;
	
	// TODO move this as into BufferedPair
	static boolean Equals(BufferedPair pair1, BufferedPair pair2) {
		return pair1.proxyId1 == pair2.proxyId1
				&& pair1.proxyId2 == pair2.proxyId2;
	}

	// TODO move this as into BufferedPair (implements Comparable)
	static boolean minor(BufferedPair pair1, BufferedPair pair2) {
		if (pair1.proxyId1 < pair2.proxyId1)
			return true;

		if (pair1.proxyId1 == pair2.proxyId1) {
			return pair1.proxyId2 < pair2.proxyId2;
		}

		return false;
	}
	
	public int compareTo(Object o){
		//We'd rather have things blow up and get a ClassCastException
		//than handle this robustly, because something's seriously
		//wrong if this is called on a non-BufferedPair object
		BufferedPair p = (BufferedPair)o;
		if (minor(this,p)) return -1;
		else if (Equals(this,p)) return 0;
		else return 1;
	}
	
}
