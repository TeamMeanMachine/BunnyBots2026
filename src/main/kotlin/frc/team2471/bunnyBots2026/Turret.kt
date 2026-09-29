package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.units.degrees

object Turret: SubsystemBase("Turret") {
    val ZERO_POSITION = 0.0.degrees

    val turretMotor = TalonFX(Talons.TURRET_0)
    val zeroLimitSwitch = DigitalInput(DigitalSensors.TURRET_ZERO_SWITCH)
    val zeroLimitSwitchPressed get() = zeroLimitSwitch.get()

    var fieldCentricSetpoint = 0.0.degrees
        set(value) {
            field = value
            turretMotor.setControl(PositionVoltage(field - Drive.heading.measure))
        }

    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
        turretMotor.addFollower(Talons.TURRET_1)
    }

    fun zero(): Command = run {
            turretMotor.setControl(DutyCycleOut(0.02))
        }.until { zeroLimitSwitchPressed }.finallyRun {
            turretMotor.setPosition(ZERO_POSITION)
            turretMotor.setControl(NeutralOut())
    }
}