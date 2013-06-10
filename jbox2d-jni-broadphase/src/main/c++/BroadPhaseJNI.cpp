#include "org_jbox2d_collision_broadphase_BroadPhaseJNI.h"
#include "b2BroadPhase.h"
#include "stdio.h"

jclass globalPhaseClass;
jfieldID nativePhaseId;

jclass globalCallbackClass;
jmethodID treeCallbackMethodId;

jclass globalRaycastClass;
jmethodID treeRaycastMethodId;

jclass aabb2class;
jfieldID aabblx;
jfieldID aabbly;
jfieldID aabbux;
jfieldID aabbuy;

jclass globalPairClass;
jmethodID pairCallbackMethodId;

b2BroadPhase* getPhase(JNIEnv *env, jobject obj) {
	//printf("Getting phase, jobject at %llx\n",
	//	(long long unsigned int) (obj - (jobject) NULL));
	b2BroadPhase* tree = (b2BroadPhase*) env->GetLongField(obj, nativePhaseId);
	//printf("After, phase at %llx\n",
	//	(long long unsigned int) (tree - (b2BroadPhase*) NULL));
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

class PairCallbackHelper {
public:
	PairCallbackHelper(JNIEnv *env, jobject callback) :
			m_env(env), m_callback(callback) {
	}
	;

	void AddPair(void* userData1, void* userData2) {
		if (pairCallbackMethodId == 0) {
			globalPairClass = m_env->FindClass(
					"org/jbox2d/callbacks/PairCallback");
			globalPairClass = (jclass) m_env->NewGlobalRef(globalPairClass);
			pairCallbackMethodId = m_env->GetMethodID(globalPairClass,
					"addPair", "(Ljava/lang/Object;Ljava/lang/Object;)V");
		}

		m_env->CallVoidMethod(m_callback, pairCallbackMethodId,
				(jobject) userData1, (jobject) userData2);
	}

private:
	JNIEnv *m_env;
	jobject m_callback;
};

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    createNative
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_createNative
(JNIEnv *env, jobject obj) {
	//printf("Creating phase\n");
	b2BroadPhase* tree;
	if (nativePhaseId == 0) {
		globalPhaseClass = env->FindClass("org/jbox2d/collision/broadphase/BroadPhaseJNI");
		globalPhaseClass = (jclass) env->NewGlobalRef(globalPhaseClass);
		nativePhaseId = env->GetFieldID( globalPhaseClass, "nativeAddress", "J" );
	}
	tree = new b2BroadPhase();
	env->SetLongField( obj, nativePhaseId, (jlong)tree);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    createProxy
 * Signature: (FFFFLjava/lang/Object;)I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_createProxy(
		JNIEnv *env, jobject obj, jfloat lowerX, jfloat lowerY, jfloat upperX,
		jfloat upperY, jobject userData) {
	b2BroadPhase* tree = getPhase(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	int32 id = tree->CreateProxy(bounds, env->NewGlobalRef(userData));
	//printf("Created proxy at %d with user data at %llx\n", id,
	//		(long long unsigned int) (userData - (jobject) NULL));
	return (jint) id;
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    destroyProxy
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_destroyProxy
(JNIEnv *env, jobject obj, jint proxy) {
	b2BroadPhase* tree = getPhase(env, obj);
	jobject data = (jobject) tree->GetUserData(proxy);
	env->DeleteGlobalRef(data);
	tree->DestroyProxy(proxy);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    moveProxy
 * Signature: (IFFFFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_moveProxy
(JNIEnv *env, jobject obj, jint proxy, jfloat lowerX, jfloat lowerY,
		jfloat upperX, jfloat upperY, jfloat dispX, jfloat dispY) {
	b2BroadPhase* tree = getPhase(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	b2Vec2 disp;
	disp.Set(dispX, dispY);
	tree->MoveProxy(proxy, bounds, disp);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    touchProxy
 * Signature: (I)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_touchProxy
(JNIEnv *env, jobject obj, jint proxy) {
	b2BroadPhase* phase = getPhase(env, obj);
	phase->TouchProxy(proxy);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getUserData
 * Signature: (I)Ljava/lang/Object;
 */
JNIEXPORT jobject JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getUserData(
		JNIEnv *env, jobject obj, jint proxy) {
	//printf("Getting user data for proxy at %d\n", proxy);
	b2BroadPhase* tree = getPhase(env, obj);
	jobject data = (jobject) tree->GetUserData(proxy);
	//printf("User data at %llx\n",
	//		(unsigned long long int) (holder->m_orig - (jobject) NULL));
	return data;
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getFat
 * Signature: (I)Lorg/jbox2d/collision/broadphase/AABB2;
 */
JNIEXPORT jobject JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getFat(
		JNIEnv *env, jobject obj, jint proxy) {
	b2BroadPhase* tree = getPhase(env, obj);
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
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    testOverlap
 * Signature: (II)Z
 */
JNIEXPORT jboolean JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_testOverlap(
		JNIEnv *env, jobject obj, jint proxyA, jint proxyB) {
	b2BroadPhase* phase = getPhase(env, obj);
	return phase->TestOverlap(proxyA, proxyB);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getProxyCount
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getProxyCount(
		JNIEnv *env, jobject obj) {
	b2BroadPhase* phase = getPhase(env, obj);
	return phase->GetProxyCount();
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    updatePairs
 * Signature: (Lorg/jbox2d/callbacks/PairCallback;)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_updatePairs
(JNIEnv *env, jobject obj, jobject callback) {
	if (callback == NULL) {
		return;
	}
	b2BroadPhase* phase = getPhase(env, obj);
	PairCallbackHelper helper(env, callback);
	phase->UpdatePairs(&helper);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    query
 * Signature: (Lorg/jbox2d/callbacks/TreeCallback;FFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_query
(JNIEnv *env, jobject obj, jobject callback, jfloat lowerX, jfloat lowerY, jfloat upperX, jfloat upperY) {
	b2BroadPhase* tree = getPhase(env, obj);
	b2AABB bounds;
	bounds.lowerBound.Set(lowerX, lowerY);
	bounds.upperBound.Set(upperX, upperY);
	TreeCallbackHelper helper(env, callback);
	tree->Query(&helper, bounds);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    raycast
 * Signature: (Lorg/jbox2d/collision/broadphase/RaycastWrapper;FFFFF)V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_raycast
(JNIEnv *env, jobject obj, jobject callback, jfloat p1x, jfloat p1y, jfloat p2x, jfloat p2y, jfloat maxFraction) {
	b2BroadPhase* tree = getPhase(env, obj);
	TreeRaycastHelper helper(env, callback);
	b2RayCastInput input;
	input.p1.Set(p1x, p1y);
	input.p2.Set(p2x, p2y);
	input.maxFraction = maxFraction;
	tree->RayCast(&helper, input);
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getTreeHeight
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getTreeHeight(
		JNIEnv *env, jobject obj) {
	b2BroadPhase* phase = getPhase(env, obj);
	return phase->GetTreeHeight();
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getTreeBalance
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getTreeBalance(
		JNIEnv *env, jobject obj) {
	b2BroadPhase* phase = getPhase(env, obj);
	return phase->GetTreeBalance();
}

/*
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    getTreeQuality
 * Signature: ()F
 */
JNIEXPORT jfloat JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_getTreeQuality(
		JNIEnv *env, jobject obj) {
	b2BroadPhase* phase = getPhase(env, obj);
	return phase->GetTreeQuality();
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
 * Class:     org_jbox2d_collision_broadphase_BroadPhaseJNI
 * Method:    freeNative
 * Signature: ()V
 */
JNIEXPORT void JNICALL Java_org_jbox2d_collision_broadphase_BroadPhaseJNI_freeNative
(JNIEnv *env, jobject obj) {
	b2BroadPhase* phase = getPhase(env, obj);
	DeleteHelper helper(env);
	phase->Iterate(&helper);
	free(phase);
}
