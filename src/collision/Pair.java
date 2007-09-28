package collision;

public class Pair {
	private final int e_bufferedPair = 0x0001;
	private final int e_removePair = 0x0002;

	Object userData;
	int proxyId1;
	int proxyId2;
	int status;

	void SetBuffered() {
		status |= e_bufferedPair;
	}

	void ClearBuffered() {
		status &= ~e_bufferedPair;
	}

	boolean IsBuffered() {
		return (status & e_bufferedPair) != 0;
	}

	void SetAdded() {
		status &= ~e_removePair;
	}

	void SetRemoved() {
		status |= e_removePair;
	}

	boolean IsRemoved() {
		return (status & e_removePair) == e_removePair;
	}
}
