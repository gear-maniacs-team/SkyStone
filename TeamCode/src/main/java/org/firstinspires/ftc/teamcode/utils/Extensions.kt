package org.firstinspires.ftc.teamcode.utils

fun rpmToTps(rpm: Double, encoder: Double) = rpm * (encoder / 60.0)

fun multiplyVelocity(velocity: Double, multiplier: Double) =
    if (multiplier > 1) velocity else velocity * multiplier
