package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.hardware.CANrange
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.hardware.loggedMotors.LoggedTalonFX

object Intake: SubsystemBase("Intake") {

    val rollerMotor = LoggedTalonFX(Talons.INTAKE_ROLLER)
    val canRange = CANrange(-1)

    @get:AutoLogOutput(key = "Intake/rollerSetpoint")
    var rollerSetpoint = 0.0
        set(value) {
            rollerMotor.setControl(DutyCycleOut(value))
            field = value
        }

    @get:AutoLogOutput(key = "Intake/currentState")
    var currentState = State.OFF

    init {
        rollerMotor.applyConfiguration {
            currentLimits(10.0, 30.0, 1.0)
        }

        rollerMotor.configSim(DCMotor.getKrakenX44(1), 0.1)

        defaultCommand = defaultCmd()
    }

    fun defaultCmd(): Command = runCommand(this) {
        when (currentState) {
            State.OFF -> {
                rollerSetpoint = 0.0
            }
            State.INTAKE -> {
                rollerSetpoint = 10.0
            }
            State.SPIT -> {
                rollerSetpoint = -10.0
            }
        }
    }

    enum class State {
        OFF,
        INTAKE,
        SPIT
    }
}