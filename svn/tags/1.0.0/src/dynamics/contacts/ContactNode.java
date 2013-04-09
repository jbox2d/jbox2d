package dynamics.contacts;

import dynamics.Body;

public class ContactNode {
    public Body other;

    public Contact contact;

    public ContactNode prev;

    public ContactNode next;
}
