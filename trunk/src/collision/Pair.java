package collision;

public class Pair implements Comparable{
	private final int e_bufferedPair = 0x0001;
	private final int e_removePair = 0x0002;

	public Object userData;
	public int proxyId1;
	public int proxyId2;
	public int status;

	public void SetBuffered() {
		status |= e_bufferedPair;
	}

	public void ClearBuffered() {
		status &= ~e_bufferedPair;
	}

	public boolean IsBuffered() {
		return (status & e_bufferedPair) != 0;
	}

	public void SetAdded() {
		status &= ~e_removePair;
	}

	public void SetRemoved() {
		status |= e_removePair;
	}

	public boolean IsRemoved() {
		return (status & e_removePair) == e_removePair;
	}
	
	public int compareTo(Object c){
		
		return 0;
	}
}
