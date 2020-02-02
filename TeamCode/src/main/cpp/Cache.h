#pragma once

#include <jni.h>

// JVM Cached Classes
namespace Cache
{
    inline jclass robotPosClass;

    jclass cacheClass(JNIEnv *env, jclass cls);
    void createCaches(JNIEnv *env);
    void cleanCaches(JNIEnv *env);
}
