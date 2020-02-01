#include "Cache.h"

namespace Cache
{
    jclass cacheClass(JNIEnv *env, jclass cls)
    {
        return static_cast<jclass>(env->NewGlobalRef(cls));
    }

    void createCaches(JNIEnv *env)
    {
        encodersResultClass = cacheClass(env, env->FindClass("net/gearmaniacs/teamcode/hardware/sensors/Encoders$Result"));
    }

    void cleanCaches(JNIEnv *env)
    {
        env->DeleteGlobalRef(encodersResultClass);
    }
}
