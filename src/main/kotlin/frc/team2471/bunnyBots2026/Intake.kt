package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.units.degrees

object Intake: SubsystemBase("Intake") {
    val rollerMotor = TalonFX(Falcons.INTAKE_ROLLER)
    var rollerMotorSetpoint = 0.0
        set(value) {
            rollerMotor.setControl(DutyCycleOut(value))
            field = value
        }
    val wristMotor = TalonFX(Falcons.INTAKE_WRIST)
    var wristMotorSetPoint = 0.0.degrees
        set(value) {
            wristMotor.setControl(PositionVoltage(value))
            field = value
        }

    init {
        rollerMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
        wristMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
    }
}