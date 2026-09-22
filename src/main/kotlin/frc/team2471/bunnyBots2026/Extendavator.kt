package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.units.asFeet
import org.team2471.frc.lib.units.feet

object Extendavator: SubsystemBase("Extendavator") {

    val elevatorMotor = TalonFX(Falcons.ELEVATOR_0)
    val extensionMotor = TalonFX(Falcons.EXTENSION)

    var heightSetpoint = 0.0.feet
        set(value) {
            elevatorMotor.setControl(PositionVoltage(value.asFeet))
            field = value
        }
    var extensionSetpoint = 0.0.feet
        set(value) {
            field = value.coerceIn(0.0.feet, 3.0.feet)
            extensionMotor.setControl(PositionVoltage(field.asFeet))
        }

    init {
        elevatorMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
        elevatorMotor.addFollower(Falcons.ELEVATOR_1)
        extensionMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
    }

    fun stow() {

    }
}