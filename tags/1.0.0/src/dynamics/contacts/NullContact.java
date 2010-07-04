package dynamics.contacts;

import java.util.List;

import collision.Manifold;

public class NullContact extends Contact {

    @Override
    public void Evaluate() {
    }
    
    public NullContact(){
        super();
    }

    public Contact clone() {
        return new NullContact();
    }

    @Override
    public List<Manifold> GetManifolds() {
         System.out.println("NullContact.GetManifolds()");
        return null;
    }
}
