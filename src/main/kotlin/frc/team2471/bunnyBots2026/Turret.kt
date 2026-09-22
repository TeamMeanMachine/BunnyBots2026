package frc.team2471.bunnyBots2026

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.hardware.ctre.addFollower
import org.team2471.frc.lib.hardware.ctre.applyConfiguration
import org.team2471.frc.lib.hardware.ctre.currentLimits
import org.team2471.frc.lib.units.degrees

object Turret: SubsystemBase("Turret") {
    val turretMotor = TalonFX(Falcons.TURRET_0)


    var fieldCentricSetpoint = 0.0.degrees

    init {
        turretMotor.applyConfiguration {
            currentLimits(30.0, 40.0, 1.0)
        }
        turretMotor.addFollower(Falcons.TURRET_1)
    }
}