#include "org_jbox2d_collision_broadphase_DynamicTreeJNI.h"
#include "b2DynamicTree.h"
#include "stdio.h"

jclass globalTreeClass;
jfieldID nativeTreeId;

jclass globalCallbackClass;
jmethodID treeCallbackMethodId;

jclass globalRaycastClass;
jmethodID treeRaycastMethodId;

jclass aabb2class;
jfieldID aabblx;
jfieldID aabbly;
jfieldID aabbux;
jfieldID aabbuy;

b2DynamicTree* getTree(JNIEnv *env, jobject obj) {
	//printf("Getting tree, jobject at %llx\n",
	//		(long long unsigned int) (obj - (jobject) NULL));
	b2DynamicTree* tree = (b2DynamicTree*) env->GetLongField(obj, nativeTreeId);
	//printf("After, tree at %llx\n",
	//		(long long unsigned int) (tree - (b2DynamicTree*) NULL));
	return tree;
}

class TreeCallbackHelper {
public:
	TreeCallbackHelper(JNIEnv *env, jobject callback) :
			m_env(env), m_callback(callback) {
	}
	;

	bool QueryCallback(int proxyId) {
		if (treeCallbackMethodId == 0) {
			globalCallbackClass = m_env->FindClass(
					"org/jbox2d/callbacks/TreeCallback");
			globalCallbackClass = (jclass) m_env->NewGlobalRef(
					globalCallbackClass);
			treeCallbackMethodId = m_env->GetMethodID(globalCallbackClass,
					"treeCallback", "(I)Z");
		}
		return m_env->CallBooleanMethod(m_callback, treeCallbackMethodId,
				(jint) proxyId);
	}

private:
	JNIEnv *m_env;
	jobject m_callback;
};

class TreeRaycastHelper {
public:
	TreeRaycastHelper(JNIEnv *env, jobject callback) :
			m_env(env), m_callback(callback) {
	}
	;

	float RayCastCallback(const b2RayCastInput &input, int nodeId) {
		if (treeRaycastMethodId == 0) {
			globalRaycastClass = m_env->FindClass(
					"org/jbox2d/collision/broadphase/RaycastWrapper");
			globalRaycastClass = (jclass) m_env->NewGlobalRef(
					globalRaycastClass);
			treeRaycastMethodId = m_env->GetMethodID(globalRaycastClass,
					"callback", "(FFFFFI)F");
		}
		const b2Vec2 &p1 = input.p1;
		const b2Vec2 &p2 = input.p2;
		return m_env->CallFloatMethod(m_callback, treeRaycastMethodId,
				(jfloat) p1.x, (jfloat) p1.y, (jfloat) p2.x, (jfloat) p2.y,
				(jfloat) input.maxFraction, (jint) nodeId);
	}

private:
	JNIEnv *m_env;
	jobject m_callback;
};

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    createNativeTree
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_createNativeTree
(JNIEnv *env, jobject obj) {
	b2DynamicTree* tree;
	if (nativeTreeId == 0) {
		globalTreeClass = env->FindClass("org/jbox2d/collision/broadphase/DynamicTreeJNI");
		globalTreeClass = (jclass) env->NewGlobalRef(globalTreeClass);
		nativeTreeId = env->GetFieldID( globalTreeClass, "nativeAddress", "J" );
	}
	tree = new b2DynamicTree();
	env->SetLongField( obj, nativeTreeId, (jlong)tree);
}
/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    createProxy
 * Signature: (FFFFLjava/lang/Object;)I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_createProxy(
		JNIEnv *env, jobject obj, jfloat lowerX, jfloat lowerY, jfloat upperX,
		jfloat upperY, jobject userData) {
	b2DynamicTree* tree = getTree(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	int32 id = tree->CreateProxy(bounds, env->NewGlobalRef(userData));
	//printf("Created proxy at %d with user data at %llx\n", id,
	//		(long long unsigned int) (userData - (jobject) NULL));
	return (jint) id;
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    destroyProxy
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_destroyProxy
(JNIEnv *env, jobject obj, jint proxy) {
	b2DynamicTree* tree = getTree(env, obj);
	jobject data = (jobject) tree->GetUserData(proxy);
	env->DeleteGlobalRef(data);
	tree->DestroyProxy(proxy);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    moveProxy
 * Signature: (IFFFFFF)Z
 */
JNIEXPORT jboolean JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_moveProxy(
		JNIEnv *env, jobject obj, jint proxy, jfloat lowerX, jfloat lowerY,
		jfloat upperX, jfloat upperY, jfloat dispX, jfloat dispY) {
	b2DynamicTree* tree = getTree(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	b2Vec2 disp;
	disp.Set(dispX, dispY);
	return tree->MoveProxy(proxy, bounds, disp);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getUserData
 * Signature: (I)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getUserData(
		JNIEnv *env, jobject obj, jint proxy) {
	//printf("Getting user data for proxy at %d\n", proxy);
	b2DynamicTree* tree = getTree(env, obj);
	jobject data = (jobject) tree->GetUserData(proxy);
	//printf("User data at %llx\n",
	//		(unsigned long long int) (holder->m_orig - (jobject) NULL));
	return data;
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    overlap
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_overlap(
		JNIEnv *env, jobject obj, jint proxyA, jint proxyB) {
	b2DynamicTree* tree = getTree(env, obj);
	return b2TestOverlap(tree->GetFatAABB(proxyA), tree->GetFatAABB(proxyB));
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    query
 * Signature: (Lorg/jbox2d/callbacks/TreeCallback;FFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_query__Lorg_jbox2d_callbacks_TreeCallback_2FFFF
(JNIEnv *env, jobject obj, jobject callback, jfloat lowerX, jfloat lowerY, jfloat upperX, jfloat upperY) {
	b2DynamicTree* tree = getTree(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	TreeCallbackHelper helper(env, callback);
	tree->Query(&helper, bounds);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    query
 * Signature: (Lorg/jbox2d/callbacks/TreeCallback;I)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_query__Lorg_jbox2d_callbacks_TreeCallback_2I
(JNIEnv *env, jobject obj, jobject callback, jint proxy) {
	b2DynamicTree* tree = getTree(env, obj);
	b2AABB bounds = tree->GetFatAABB(proxy);
	TreeCallbackHelper helper(env, callback);
	tree->Query(&helper, bounds);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    raycast
 * Signature: (Lorg/jbox2d/callbacks/TreeRayCastCallback;FFFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_raycast
(JNIEnv *env, jobject obj, jobject callback, jfloat p1x, jfloat p1y, jfloat p2x, jfloat p2y, jfloat maxFraction) {
	b2DynamicTree* tree = getTree(env, obj);
	TreeRaycastHelper helper(env, callback);
	b2RayCastInput input;
	input.p1.Set(p1x, p1y);
	input.p2.Set(p2x, p2y);
	input.maxFraction = maxFraction;
	tree->RayCast(&helper, input);
}

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    getFat
 * Signature: (I)Lorg/jbox2d/collision/broadphase/AABB2;
 */
JNIEXPORT jobject JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_getFat(
		JNIEnv *env, jobject obj, jint proxy) {
	b2DynamicTree* tree = getTree(env, obj);
	b2AABB aabb = tree->GetFatAABB(proxy);
	if (aabb2class == 0) {
		aabb2class = env->FindClass("org/jbox2d/collision/broadphase/AABB2");
		aabb2class = (jclass) env->NewGlobalRef(aabb2class);
		aabblx = env->GetFieldID(aabb2class, "lx", "F");
		aabbly = env->GetFieldID(aabb2class, "ly", "F");
		aabbux = env->GetFieldID(aabb2class, "ux", "F");
		aabbuy = env->GetFieldID(aabb2class, "uy", "F");
	}
	jobject ret = env->AllocObject(aabb2class);
	env->SetFloatField(ret, aabblx, aabb.lowerBound.x);
	env->SetFloatField(ret, aabbly, aabb.lowerBound.y);
	env->SetFloatField(ret, aabbux, aabb.upperBound.x);
	env->SetFloatField(ret, aabbuy, aabb.upperBound.y);
	return ret;
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

class DeleteHelper {
public:
	DeleteHelper(JNIEnv *env) :
			m_env(env) {
	}
	;

	void process(void* userData) {
		m_env->DeleteGlobalRef((jobject) userData);
	}

private:
	JNIEnv *m_env;
};

/*
 * Class:     org_jbox2d_collision_broadphase_DynamicTreeJNI
 * Method:    freeNative
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_DynamicTreeJNI_freeNative
(JNIEnv *env, jobject obj) {
	b2DynamicTree* tree = getTree(env, obj);
	DeleteHelper helper(env);
	tree->Iterate(&helper);
	free(tree);
}
