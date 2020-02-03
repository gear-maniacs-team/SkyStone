#include "Encoders.h"

#include "../Cache.h"

namespace
{
    Encoders encoders{};

    jfieldID currentXId = nullptr;
    jfieldID currentYId = nullptr;
    jfieldID currentAngleId = nullptr;
}

extern "C" JNIEXPORT void JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_initNative(JNIEnv *pEnv, jobject /*instance*/)
{
    currentXId = pEnv->GetStaticFieldID(Cache::robotPosClass, "currentX", "D");
    currentYId = pEnv->GetStaticFieldID(Cache::robotPosClass, "currentY", "D");
    currentAngleId = pEnv->GetStaticFieldID(Cache::robotPosClass, "currentAngle", "D");

    encoders.init();
}

extern "C" JNIEXPORT void JNICALL
Java_net_gearmaniacs_teamcode_hardware_sensors_Encoders_updateNative(
        JNIEnv *pEnv, jobject /*instance*/,
        jdouble leftPos, jdouble rightPos, jdouble backPos)
{
    using namespace Cache;

    const double currentX = pEnv->GetStaticDoubleField(robotPosClass, currentXId);
    const double currentY = pEnv->GetStaticDoubleField(robotPosClass, currentYId);
    const double currentAngle = pEnv->GetStaticDoubleField(robotPosClass, currentAngleId);

    const auto[deltaX, deltaY, deltaAngle] =
        encoders.update({ double(leftPos), double(rightPos), double(backPos) }, currentAngle);

    pEnv->SetStaticDoubleField(robotPosClass, currentXId, currentX + deltaX);
    pEnv->SetStaticDoubleField(robotPosClass, currentYId, currentY + deltaY);
    pEnv->SetStaticDoubleField(robotPosClass, currentAngleId, currentAngle + deltaAngle);
}
