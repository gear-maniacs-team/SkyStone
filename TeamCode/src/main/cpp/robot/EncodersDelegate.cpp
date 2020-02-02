#include "Encoders.h"

#include "../Cache.h"

namespace
{
    Encoders encoders{};
    jmethodID encodersResultConstructorId = nullptr;
}

extern "C" JNIEXPORT void JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_initNative(JNIEnv *pEnv, jobject/*instance*/)
{
    encoders.init();
    encodersResultConstructorId = pEnv->GetMethodID(Cache::encodersResultClass, "<init>", "(DDD)V");
}

extern "C" JNIEXPORT jobject JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_updateNative(
        JNIEnv *pEnv, jobject /*instance*/,
        jdouble leftPos, jdouble rightPos, jdouble backPos, jdouble currentAngle)
{
    const RobotPose pose{ double(leftPos), double(rightPos), double(backPos) };
    const auto[deltaX, deltaY, deltaAngle] = encoders.update(pose, double(currentAngle));


    return pEnv->NewObject(Cache::encodersResultClass, encodersResultConstructorId,
                           deltaX, deltaY, deltaAngle);
}
