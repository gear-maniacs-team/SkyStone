#include "Cache.h"

namespace Cache
{
    jclass cacheClass(JNIEnv *env, jclass cls)
    {
        return static_cast<jclass>(env->NewGlobalRef(cls));
    }

    void createCaches(JNIEnv *env)
    {
        robotPosClass = cacheClass(env, env->FindClass("net/gearmaniacs/teamcode/RobotPos"));
    }

    void cleanCaches(JNIEnv *env)
    {
        env->DeleteGlobalRef(robotPosClass);
    }
}
