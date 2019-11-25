package org.firstinspires.ftc.teamcode.utils

fun mapToRange(inputStart: Double, inputEnd: Double, outputStart: Double, outputEnd: Double, input: Double) =
    outputStart + ((outputEnd - outputStart) / (inputEnd - inputStart)) * (input - inputStart)
