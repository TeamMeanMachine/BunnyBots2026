package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.commands.onlyRunWhileFalse
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.ctre.sensorToMechanismRatio
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asDegreesPerSecond
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.degreesPerSecond
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.seconds

object Extendavator: SubsystemBase("Extendavator") {
    val elevatorMotor = TalonFX(Talons.ELEVATOR_0)
    val extensionMotor = TalonFX(Talons.EXTENSION)
    val pivotMotor = TalonFX(Talons.INTAKE_PIVOT)

    val MAX_ELEVATOR_VELOCITY = 2.0.inchesPerSecond
    val MAX_EXTENSION_VELOCITY = 2.0.inchesPerSecond
    val MAX_PIVOT_VELOCITY = 2.0.degreesPerSecond

    var heightSetpoint = 0.0.feet
        get() = elevatorMotor.closedLoopReference.valueAsDouble.inches
        set(value) {
            field = value.coerceIn(0.0.inches, 59.25.inches)
            elevatorMotor.setControl(PositionVoltage(field.asInches))
        }

    var extensionSetpoint = 0.0.feet
        get() = extensionMotor.closedLoopReference.valueAsDouble.inches
        set(value) {
            field = value.coerceIn(0.0.feet, 3.0.feet)
            extensionMotor.setControl(PositionVoltage(field.asInches))
        }

    var pivotSetpoint = 0.0.degrees
        get() = pivotMotor.closedLoopReference.valueAsDouble.rotations
        set(value) {
            pivotMotor.setControl(PositionVoltage(value))
            field = value
        }

    val elevatorHeight: Distance get() = elevatorMotor.position.valueAsDouble.inches
    val extensionDistance: Distance get() = extensionMotor.position.valueAsDouble.inches
    val pivotAngle: Angle get() = pivotMotor.position.value

    val heightError: Distance get() = elevatorMotor.closedLoopError.valueAsDouble.inches
    val extensionError: Distance get() = extensionMotor.closedLoopError.valueAsDouble.inches
    val pivotError: Angle get() = pivotMotor.closedLoopError.valueAsDouble.rotations

    val totalErrorPose = Pose(heightError, extensionError, pivotError)
    val atSetpoint: Boolean
        get() {
            val e = totalErrorPose.absoluteValue()
            return e.pivotAngle < 1.0.degrees && e.elevatorHeight < 1.0.inches && e.extensionDistance < 1.0.inches
        }

    val heightMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value
    val extensionMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value
    val pivotMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value

    val atMotionMagicSetpoint get() = heightMMAtSetpoint && extensionMMAtSetpoint && pivotMMAtSetpoint

    init {
        elevatorMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            sensorToMechanismRatio(5.33)
        }
        elevatorMotor.addFollower(Talons.ELEVATOR_1)
        extensionMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            //        Motor to sprocket ratio * 12 tooth sprocket * 0.25in chain link length
            sensorToMechanismRatio((14.0/40.0) * 12.0 * 0.25)
        }
        pivotMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            sensorToMechanismRatio(33.75)
        }
    }

    fun stow(): Command = animateToPose(Pose.DRIVE)


    fun goToPose(pose: Pose) {
        heightSetpoint = pose.elevatorHeight
        extensionSetpoint = pose.extensionDistance
        pivotSetpoint = pose.pivotAngle
    }

    fun goToPoseWithTime(pose: Pose, maxTime: Time) {
        val difference = pose - Pose.current
        val elevatorVelocity = difference.elevatorHeight / maxTime
        val extensionVelocity = difference.extensionDistance / maxTime
        val pivotVelocity = difference.pivotAngle / maxTime

        elevatorMotor.setControl(PositionVoltage(pose.elevatorHeight.asInches).withVelocity(elevatorVelocity.asInchesPerSecond))
        extensionMotor.setControl(PositionVoltage(pose.elevatorHeight.asInches).withVelocity(extensionVelocity.asInchesPerSecond))
        pivotMotor.setControl(PositionVoltage(pose.pivotAngle).withVelocity(pivotVelocity))
    }


    fun animateToPose(pose: Pose) = run {
        val difference = pose - Pose.current
        val timeForElevator = difference.elevatorHeight / MAX_ELEVATOR_VELOCITY
        val timeForExtension = difference.extensionDistance / MAX_EXTENSION_VELOCITY
        val timeForPivot = (difference.pivotAngle.asDegrees / MAX_PIVOT_VELOCITY.asDegreesPerSecond).seconds // TODO: PR WPILIB `difference.pivotAngle / MAX_PIVOT_VELOCITY`
        val maxTime = maxOf(timeForElevator, timeForExtension, timeForPivot)
        goToPoseWithTime(pose, maxTime)



    }.onlyRunWhileFalse { atMotionMagicSetpoint }

}