package dynamics.contacts;

import collision.ShapeType;

public class ContactRegister {
    // typedef b2Contact* ContactCreateFcn(b2Shape* shape1, b2Shape* shape2,
    // b2BlockAllocator* allocator);

    // typedef void ContactDestroyFcn(b2Contact* contact, b2BlockAllocator*
    // allocator);
    ShapeType s1, s2;

    ContactCreator createFcn;

    boolean primary;
}
