package org.firstinspires.ftc.teamcode.utils

fun mapToRange(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)

fun getVelocityForRpmAndEncoderCycles(rpm: Double, encoder: Double) = rpm * (encoder / 60.0)

fun multiplyVelocity(velocity: Double, multiplier: Double) = if (multiplier > 1) velocity else velocity * multiplier
