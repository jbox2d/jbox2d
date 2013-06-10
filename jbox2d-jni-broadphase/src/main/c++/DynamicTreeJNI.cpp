#include "org_jbox2d_collision_broadphase_DynamicTreeJNI.h"
#include "b2DynamicTree.h"

jfieldID nativeTreeId;

b2DynamicTree* getTree(JNIEnv *env, jobject obj) {
	return (b2DynamicTree*) env->GetLongField(obj, nativeTreeId);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    createNativeTree
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_createNativeTree
(JNIEnv *env, jobject obj) {
	b2DynamicTree* tree;
	if (nativeTreeId == 0) {
		jclass tempClass = env->FindClass("org/jbox2d/collision/broadphase/DynamicTreeJNI");
		nativeTreeId = env->GetFieldID( tempClass, "nativeAddress", "J" );
	}
	tree = new b2DynamicTree();
	env->SetLongField( obj, nativeTreeId, (jlong)tree);
}
/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    createProxy
 * Signature: (FFFFLjava/lang/Object;)I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_createProxy
  (JNIEnv *, jobject, jfloat, jfloat, jfloat, jfloat, jobject);

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    destroyProxy
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_destroyProxy
  (JNIEnv *, jobject, jint);

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    moveProxy
 * Signature: (IFFFFFF)Z
 */
JNIEXPORT jboolean JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_moveProxy
  (JNIEnv *, jobject, jint, jfloat, jfloat, jfloat, jfloat, jfloat, jfloat);

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getUserData
 * Signature: (I)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getUserData
  (JNIEnv *, jobject, jint);

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    query
 * Signature: (Lorg/jbox2d/callbacks/TreeCallback;FFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_query
  (JNIEnv *, jobject, jobject, jfloat, jfloat, jfloat, jfloat);

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    raycast
 * Signature: (Lorg/jbox2d/callbacks/TreeRayCastCallback;Lorg/jbox2d/collision/RayCastInput;)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_raycast
(JNIEnv *, jobject, jobject, jobject) {

}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    computeHeight
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_computeHeight(
		JNIEnv *env, jobject obj) {
	return getTree(env, obj)->GetHeight();
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getHeight
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getHeight(
		JNIEnv *env, jobject obj) {
	return getTree(env, obj)->GetHeight();
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getMaxBalance
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getMaxBalance(
		JNIEnv *env, jobject obj) {
	return getTree(env, obj)->GetMaxBalance();
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getAreaRatio
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getAreaRatio(
		JNIEnv *env, jobject obj) {
	return getTree(env, obj)->GetAreaRatio();
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getInsertionCount
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getInsertionCount(
		JNIEnv *env, jobject obj) {
	return 0;
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    freeNative
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_freeNative
  (JNIEnv *env, jobject obj) {
	free(getTree(env, obj));
}
