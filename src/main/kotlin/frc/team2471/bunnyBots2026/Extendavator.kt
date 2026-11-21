package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.ctre.motionMagic
import org.team2471.frc.lib.hardware.ctre.p
import org.team2471.frc.lib.hardware.ctre.sensorToMechanismRatio
import org.team2471.frc.lib.hardware.loggedMotors.LoggedTalonFX
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asDegreesPerSecond
import org.team2471.frc.lib.units.asDegreesPerSecondPerSecond
import org.team2471.frc.lib.units.asInches
import org.team2471.frc.lib.units.asInchesPerSecond
import org.team2471.frc.lib.units.asInchesPerSecondPerSecond
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.degreesPerSecond
import org.team2471.frc.lib.units.degreesPerSecondPerSecond
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches
import org.team2471.frc.lib.units.inchesPerSecond
import org.team2471.frc.lib.units.inchesPerSecondPerSecond
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.seconds

object Extendavator: SubsystemBase("Extendavator") {
    val elevatorMotor = LoggedTalonFX(Talons.ELEVATOR_0)
    val extensionMotor = LoggedTalonFX(Talons.EXTENSION)
    val pivotMotor = LoggedTalonFX(Talons.INTAKE_PIVOT)

    val MAX_ELEVATOR_VELOCITY = 30.0.inchesPerSecond
    val MAX_EXTENSION_VELOCITY = 30.0.inchesPerSecond
    val MAX_PIVOT_VELOCITY = 50.0.degreesPerSecond

    val MAX_ELEVATOR_ACCELERATION = 10.0.inchesPerSecondPerSecond
    val MAX_EXTENSION_ACCELERATION = 10.0.inchesPerSecondPerSecond
    val MAX_PIVOT_ACCELERATION = 2.0.degreesPerSecondPerSecond

    @get:AutoLogOutput(key = "Extendavator/heightSetpoint")
    var heightSetpoint = 0.0.feet
        private set

    fun setHeight(value: Distance, velocity: LinearVelocity? = null) {
        val safeValue = value.coerceIn(0.0.inches, 59.25.inches)
        if (velocity != null) {
            elevatorMotor.setControl(PositionVoltage(safeValue.asInches).withVelocity(velocity.asInchesPerSecond))
        } else {
            elevatorMotor.setControl(MotionMagicVoltage(safeValue.asInches))
        }
        heightSetpoint = safeValue
    }

    const val MIN_EXTENSION_DIST = 22.0 // inches
    @get:AutoLogOutput(key = "Extendavator/extensionSetpoint")
    var extensionSetpoint = MIN_EXTENSION_DIST.inches
        private set

    fun setExtension(value: Distance, velocity: LinearVelocity? = null) {
        val safeValue = value.coerceIn(MIN_EXTENSION_DIST.inches, 3.5.feet)
        if (velocity != null) {
            extensionMotor.setControl(PositionVoltage(safeValue.asInches).withVelocity(velocity.asInchesPerSecond))
        } else {
            extensionMotor.setControl(MotionMagicVoltage(safeValue.asInches))
        }
//        println("set exension $safeValue")
        extensionSetpoint = safeValue
    }

    @get:AutoLogOutput(key = "Extendavator/extensionSetpoint")
    var pivotSetpoint = 0.0.degrees
        private set

    fun setPivot(value: Angle, velocity: AngularVelocity? = null) {
        val safeValue = value.coerceIn(-45.0.degrees, 45.0.degrees)
        if (velocity != null) {
            pivotMotor.setControl(PositionVoltage(safeValue).withVelocity(velocity))
        } else {
            pivotMotor.setControl(MotionMagicVoltage(safeValue))
        }
        pivotSetpoint = safeValue
    }

    @get:AutoLogOutput(key = "Extendavator/elevatorHeight")
    val elevatorHeight: Distance get() = elevatorMotor.position.valueAsDouble.inches
    @get:AutoLogOutput(key = "Extendavator/extensionDistance")
    val extensionDistance: Distance get() = extensionMotor.position.valueAsDouble.inches
    @get:AutoLogOutput(key = "Extendavator/pivotAngle")
    val pivotAngle: Angle get() = pivotMotor.position.value

    val heightError: Distance get() = elevatorMotor.closedLoopError.valueAsDouble.inches
    val extensionError: Distance get() = extensionMotor.closedLoopError.valueAsDouble.inches
    val pivotError: Angle get() = pivotMotor.closedLoopError.valueAsDouble.rotations

    val totalErrorPose = Pose(heightError, extensionError, pivotError)
    val atSetpoint: Boolean
        get() {
            val error = totalErrorPose.absoluteValue()
            return error.pivotAngle < 1.0.degrees && error.elevatorHeight < 1.0.inches && error.extensionDistance < 1.0.inches
        }

    val heightMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value
    val extensionMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value
    val pivotMMAtSetpoint get() = elevatorMotor.motionMagicAtTarget.value

    val atMotionMagicSetpoint get() = heightMMAtSetpoint && extensionMMAtSetpoint && pivotMMAtSetpoint

    init {
        zeroExtension()

        elevatorMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            sensorToMechanismRatio(5.33)

            motionMagic(MAX_ELEVATOR_VELOCITY.asInchesPerSecond, MAX_ELEVATOR_ACCELERATION.asInchesPerSecondPerSecond)

            p(0.4)
        }
        elevatorMotor.addFollower(Talons.ELEVATOR_1)

        elevatorMotor.configSim(DCMotor.getKrakenX60(2), 0.01)

        extensionMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            //        Motor to sprocket ratio * 12 tooth sprocket * 0.25in chain link length
            sensorToMechanismRatio((14.0/40.0) * 12.0 * 0.25)

            motionMagic(MAX_EXTENSION_VELOCITY.asInchesPerSecond, MAX_EXTENSION_ACCELERATION.asInchesPerSecondPerSecond)

            p(0.4)
        }

        extensionMotor.configSim(DCMotor.getKrakenX44(1), 0.01)

        pivotMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
            sensorToMechanismRatio(33.75)

            motionMagic(MAX_PIVOT_VELOCITY.asDegreesPerSecond, MAX_PIVOT_ACCELERATION.asDegreesPerSecondPerSecond)

            p(0.4)
        }

        pivotMotor.configSim(DCMotor.getKrakenX44(1), 0.01)
    }

    fun stow() = goToPose(Pose.DRIVE)

    fun goToPose(pose: Pose, instant: Boolean = true) {
        if (instant) {
            goToPoseWithTime(pose, calculateAnimationTime(pose))
        } else {
            goToPoseInstant(pose)
        }
    }

    fun goToPoseWithTime(pose: Pose, maxTime: Time) {
        val difference = pose - Pose.current
        val elevatorVelocity = difference.elevatorHeight / maxTime
        val extensionVelocity = difference.extensionDistance / maxTime
        val pivotVelocity = difference.pivotAngle / maxTime

        setHeight(pose.elevatorHeight, elevatorVelocity)
        setExtension(pose.extensionDistance, extensionVelocity)
        setPivot(pose.pivotAngle, pivotVelocity)
    }

    fun goToPoseInstant(pose: Pose) {
        setHeight(pose.elevatorHeight)
        setExtension(pose.extensionDistance)
        setPivot(pose.pivotAngle)
    }

    fun calculateAnimationTime(setpointPose: Pose, currentPose: Pose = Pose.current): Time {
        val difference = setpointPose - currentPose
        val timeForElevator = difference.elevatorHeight / MAX_ELEVATOR_VELOCITY
        val timeForExtension = difference.extensionDistance / MAX_EXTENSION_VELOCITY
        val timeForPivot = (difference.pivotAngle.asDegrees / MAX_PIVOT_VELOCITY.asDegreesPerSecond).seconds
        return maxOf(timeForElevator, timeForExtension, timeForPivot)
    }


    fun animateToPoseCmd(pose: Pose): Command = run {
        goToPoseWithTime(pose, calculateAnimationTime(pose))
    }.until { atSetpoint }

    fun zeroExtension() {
        extensionMotor.setPosition(MIN_EXTENSION_DIST)
    }


    fun simpleAimAtTower(tower: () -> FieldManager.Tower): Command {
        return runCommand(this) {
            val pose = Pose.generateScorePoseForTower(tower())
            val time = calculateAnimationTime(pose)
            goToPoseWithTime(pose, time)
        }
    }

}