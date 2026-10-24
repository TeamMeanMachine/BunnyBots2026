package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.printlnCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitUntilCommand
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.unWrap
import org.team2471.frc.lib.util.translation

object Turret: SubsystemBase("Turret") {
    val ZERO_POSITION = 0.0.degrees

    val turretMotor = TalonFX(Talons.TURRET_0)
    val zeroLimitSwitch = DigitalInput(DigitalSensors.TURRET_ZERO_SWITCH)
    val zeroLimitSwitchPressed get() = zeroLimitSwitch.get()

    // TODO: factor in turret offset from drivebase if applicable

    @get:AutoLogOutput(key = "Turret/robotCentricSetpoint")
    var robotCentricSetpoint = 0.0.degrees
        set(value) {
            field = value.unWrap(turretMotor.position.value)
            turretMotor.setControl(PositionVoltage(field))
        }

    fun goToFieldCentricAngle(angle: Angle) {
        robotCentricSetpoint = (angle - Drive.heading.measure)
    }


    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
        turretMotor.addFollower(Talons.TURRET_1)
    }

    override fun periodic() {
        Logger.recordOutput("Turret/turretPose", Pose2d(Drive.localizer.pose.translation, robotCentricSetpoint.asRotation2d + Drive.heading))
    }

    fun pointTowardDriveTranslation(): Command = run {
        val joystickTranslation = Drive.getChassisSpeedsFromJoystick().translation
        if (joystickTranslation.norm > 0.05) {
            goToFieldCentricAngle(joystickTranslation.angle.measure)
        }
    }

    fun zero(): Command {
        return sequenceCommand(
            printlnCommand("Zeroing turret"),
            runOnceCommand { turretMotor.setControl(DutyCycleOut(-0.1)) },
            waitUntilCommand { zeroLimitSwitchPressed },
            waitUntilCommand { !zeroLimitSwitchPressed },
            runOnceCommand { turretMotor.setControl(DutyCycleOut(0.02)) },
            waitUntilCommand { zeroLimitSwitchPressed }
        ).finallyRun {
            turretMotor.setPosition(ZERO_POSITION)
            turretMotor.setControl(NeutralOut())
            println("Zeroed turret")
        }.apply {
            addRequirements(Turret)
        }//.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
    }


}