package collision;

public abstract class PairCallback {
    // This should return the new pair user data. It is okay if the
    // user data is null.
    public abstract Object PairAdded(Object proxyUserData1,
            Object proxyUserData2);

    // This should free the pair's user data. In extreme circumstances, it is possible
    // this will be called with null pairUserData because the pair never existed.
    public abstract void PairRemoved(Object proxyUserData1,
            Object proxyUserData2, Object pairUserData);
}
