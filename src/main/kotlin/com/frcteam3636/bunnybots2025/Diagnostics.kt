package com.frcteam3636.bunnybots2025

import com.ctre.phoenix6.CANBus
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Drivetrain
import com.frcteam3636.bunnybots2025.subsystems.drivetrain.Gyro
import com.frcteam3636.bunnybots2025.utils.cachedStatus
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.GenericHID

/**
 * Reports diagnostics and sends notifications to the driver station.
 *
 * Each diagnostic condition is stored as a boolean value, and alerts are generated when one
 * becomes problematic. The alerts are sent to the driver dashboard and logged to the console.
 */
object Diagnostics {
    sealed class RobotAlert(message: String, alertType: AlertType = AlertType.kError) {
        val alert = Alert(message, alertType)

        object GyroDisconnected : RobotAlert("Failed to connect to gyro, vision and odometry will likely not function.")
        object LimelightDisconnected :
            RobotAlert("Failed to connect to one or more LimeLights, vision will be impaired.")

        object DubiousAutoChoice :
            RobotAlert(
                "There is no auto selected. Are you absolutely sure you **do not** want to run an auto?",
                AlertType.kWarning
            )

        object JoystickDisconnected :
            RobotAlert("One or more Joysticks have disconnected, driver controls will not work.")

        object ControllerDisconnected :
            RobotAlert("An Xbox Controller has disconnected, operator controls will not work.")

        object HIDDeviceIsWrongType :
            RobotAlert(
                "Check USB device order in Driver Station! The connected devices are likely in the wrong order.",
                AlertType.kWarning
            )

        object CANivoreRefreshFailed : RobotAlert(
            "A CANivore refresh failed, outdated data is being received from CANivore devices..",
            AlertType.kError
        )

        class CAN private constructor(bus: CANBus) {
            private class BusFailure(bus: CANBus) : RobotAlert("The \"${bus.humanReadableName}\" CAN bus has FAILED!")
            private class BusError(bus: CANBus) :
                RobotAlert("Devices on the \"${bus.humanReadableName}\" CAN bus are experiencing errors.")

            val failure: RobotAlert = BusFailure(bus)
            val error: RobotAlert = BusError(bus)

            companion object {
                private val knownBuses = HashMap<CANBus, CAN>()
                fun bus(bus: CANBus): CAN = knownBuses.getOrPut(bus) { CAN(bus) }
            }
        }
    }

    private var robotAlerts = HashSet<RobotAlert>()

    fun reset() {
        robotAlerts.clear()
    }

    fun reportAlert(robotAlert: RobotAlert) {
        robotAlerts += robotAlert
    }

    /** Report the CAN Bus's errors */
    fun report(canBus: CANBus) {
        val status = canBus.cachedStatus

        // Can't connect to the CAN Bus at all? It's probably unplugged or might have even failed.
        if (status.Status.isError) {
            reportAlert(RobotAlert.CAN.bus(canBus).failure)
            return
        }

        if (status.REC + status.TEC > 0) {
            reportAlert(RobotAlert.CAN.bus(canBus).error)
        }
    }

    fun report(gyro: Gyro) {
        if (!gyro.connected) {
            reportAlert(RobotAlert.GyroDisconnected)
        }
    }

    fun reportDSPeripheral(controller: GenericHID, isController: Boolean) {
        if (!controller.isConnected) {
            if (isController) {
                reportAlert(RobotAlert.ControllerDisconnected)
            } else {
                reportAlert(RobotAlert.JoystickDisconnected)
            }
            return
        }

        val type = controller.type
        val isExpectedType = if (isController) {
            type == GenericHID.HIDType.kHIDGamepad || type == GenericHID.HIDType.kXInputGamepad
        } else {
            type == GenericHID.HIDType.kHIDJoystick || type == GenericHID.HIDType.kHIDFlight
        }

        if (!isExpectedType) {
            reportAlert(RobotAlert.HIDDeviceIsWrongType)
        }
    }

    fun periodic() {
        reset()

        if (!Drivetrain.allPoseProvidersConnected) {
            reportAlert(RobotAlert.LimelightDisconnected)
        }

        if (!Robot.didRefreshSucceed) {
            reportAlert(RobotAlert.CANivoreRefreshFailed)
        }

        // Only run these while disabled to save loop times
        if (!Robot.isDisabled) {
            if (Robot.autoChooser.selected == "Nothing")
                reportAlert(RobotAlert.DubiousAutoChoice)
        }
    }

    private var previousRobotAlerts = HashSet<RobotAlert>()

    /** Show pending alerts. */
    fun send() {
        for (robotAlert in previousRobotAlerts) {
            robotAlert.alert.set(false)
        }
        previousRobotAlerts.clear()

        for (robotAlert in robotAlerts) {
            robotAlert.alert.set(true)
        }

        previousRobotAlerts.addAll(robotAlerts)
    }
}

val CANBus.humanReadableName: String
    get() = if (name == "*") {
        "Canivore"
    } else {
        name
    }
