package com.frcteam3636.bunnybots2025

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.revrobotics.spark.SparkFlex
import com.revrobotics.spark.SparkLowLevel
import edu.wpi.first.wpilibj.PowerDistribution

val canivoreBus = CANBus("*")
val rioCANBus = CANBus("rio")

enum class CTREDeviceId(val num: Int, val bus: CANBus) {
    FrontLeftDrivingMotor(1, canivoreBus),
    BackLeftDrivingMotor(2, canivoreBus),
    BackRightDrivingMotor(3, canivoreBus),
    FrontRightDrivingMotor(4, canivoreBus),

    FrontLeftTurningMotor(5, canivoreBus),
    BackLeftTurningMotor(6, canivoreBus),
    BackRightTurningMotor(7, canivoreBus),
    FrontRightTurningMotor(8, canivoreBus),

    FrontLeftTurningEncoder(9, canivoreBus),
    BackLeftTurningEncoder(10, canivoreBus),
    BackRightTurningEncoder(11, canivoreBus),
    FrontRightTurningEncoder(12, canivoreBus),

    ShooterPivotMotor(13, canivoreBus),
    IntakePivotMotor(14, canivoreBus),
    IntakePivotEncoder(15, canivoreBus),
    ShooterPivotEncoder(16, canivoreBus),
    CANRangeShooter(17, canivoreBus),
    CANRangeIndexer(18, canivoreBus),
    PigeonGyro(20, canivoreBus),
}

fun CANcoder(id: CTREDeviceId) = CANcoder(id.num, id.bus)
fun TalonFX(id: CTREDeviceId) = TalonFX(id.num, id.bus)
fun Pigeon2(id: CTREDeviceId) = Pigeon2(id.num, id.bus)
fun CANrange(id: CTREDeviceId) = CANrange(id.num, id.bus)

enum class REVDeviceId(val num: Int) {
    PowerDistributionHub(1),
    UpperShooterMotor(2),
    LowerShooterMotor(3),
    IndexerMotor(4),
    IntakeMotor(5),
    ShooterFeederMotor(6),
}

fun SparkFlex(id: REVDeviceId, type: SparkLowLevel.MotorType) =
    SparkFlex(id.num, type)

fun PowerDistribution(id: REVDeviceId): PowerDistribution =
    PowerDistribution(id.num, PowerDistribution.ModuleType.kRev)