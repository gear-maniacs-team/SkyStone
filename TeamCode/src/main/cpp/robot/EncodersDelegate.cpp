#include "Encoders.h"

#include "../Cache.h"

static Encoders &getEncodersInstance()
{
    static Encoders encoders{};
    return encoders;
}

extern "C" JNIEXPORT void JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_initNative(JNIEnv *, jobject)
{
    Encoders &encoders = getEncodersInstance();
    encoders.init();
}

extern "C" JNIEXPORT jobject JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_updateNative(
        JNIEnv *pEnv, jobject /*instance*/,
        jdouble leftPos, jdouble rightPos, jdouble backPos, jdouble currentAngle)
{
    Encoders &encoders = getEncodersInstance();

    const RobotPose pose{ double(leftPos), double(rightPos), double(backPos) };
    const auto[deltaX, deltaY, deltaAngle] = encoders.update(pose, double(currentAngle));

    const static auto constructorId = pEnv->GetMethodID(Cache::encodersResultClass, "<init>", "(DDD)V");

    return pEnv->NewObject(Cache::encodersResultClass, constructorId,
                           deltaX, deltaY, deltaAngle);
}
