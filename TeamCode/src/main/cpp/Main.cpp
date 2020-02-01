#include "Cache.h"
#include "Log.h"

extern "C" JNIEXPORT jint JNI_OnLoad(JavaVM *vm, void */*reserved*/)
{
    LOGV("GearManiacs", "JNI_OnLoad");

    JNIEnv *env;
    if (vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6) != JNI_OK)
        return -1;

    Cache::createCaches(env);

    return JNI_VERSION_1_6;
}

extern "C" JNIEXPORT void JNI_OnUnload(JavaVM *vm, void */*reserved*/)
{
    LOGV("GearManiacs", "JNI_OnUnload");

    JNIEnv *env;
    vm->GetEnv(reinterpret_cast<void **>(&env), JNI_VERSION_1_6);

    // Clean the caches
    Cache::cleanCaches(env);
}
