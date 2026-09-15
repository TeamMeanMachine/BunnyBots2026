package frc.team2471.bunnyBots2026

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.units.feet

object Extendavator: SubsystemBase("Extendavator") {



    var heightSetpoint = 0.0.feet
    var extensionSetpoint = 0.0.feet
        set(value) {
            field = value.coerceIn(0.0.feet, 3.0.feet)
        }


    fun stow() {

    }
}