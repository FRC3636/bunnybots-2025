package com.frcteam3636.bunnybots2025.utils.math

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

const val TAU = PI * 2

fun Translation2d.fromPolar(magnitude: Double, angle: Double): Translation2d {
    return Translation2d(magnitude * cos(angle), magnitude * sin(angle))
}

fun Translation2d.dot(other: Translation2d): Double {
    return x * other.x + y * other.y
}

fun InterpolatingDoubleTreeMap.putVelocity(key: Distance, value: AngularVelocity) {
    put(key.inMeters(), value.inRadiansPerSecond())
}

fun InterpolatingDoubleTreeMap.getVelocity(key: Distance): AngularVelocity {
    return get(key.inMeters()).radiansPerSecond
}

fun InterpolatingDoubleTreeMap.putAngle(key: Distance, value: Angle) {
    put(key.inMeters(), value.inDegrees())
}

fun InterpolatingDoubleTreeMap.getAngle(key: Distance): Angle {
    return get(key.inMeters()).degrees
}