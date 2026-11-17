package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.loggedMotors.LoggedTalonFX

object Intake: SubsystemBase("Intake") {
    val rollerMotor = LoggedTalonFX(Talons.INTAKE_ROLLER)
    var rollerSetpoint = 0.0
        set(value) {
            rollerMotor.setControl(DutyCycleOut(value))
            field = value
        }

    var currentState = State.OFF

    init {
        rollerMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }

        rollerMotor.configSim(DCMotor.getKrakenX44(1), 0.1)
    }

    override fun periodic() {
        when (currentState) {
            State.OFF -> {
                rollerSetpoint = 0.0
            }
            State.INTAKING -> {
                rollerSetpoint = 10.0
            }
            State.SPITTING -> {
                rollerSetpoint = -10.0
            }
        }
    }

    enum class State {
        OFF,
        INTAKING,
        SPITTING
    }
}