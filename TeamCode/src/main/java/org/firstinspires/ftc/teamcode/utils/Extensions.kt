package org.firstinspires.ftc.teamcode.utils

fun multiplyVelocity(velocity: Double, multiplier: Double) =
    if (multiplier > 1) velocity else velocity * multiplier
