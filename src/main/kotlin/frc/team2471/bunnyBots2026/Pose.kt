package frc.team2471.bunnyBots2026

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import org.team2471.frc.lib.units.absoluteValue
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.inches

data class Pose(val elevatorHeight: Distance, val extensionDistance: Distance, val pivotAngle: Angle) {
    operator fun minus(other: Pose) = Pose(this.elevatorHeight - other.elevatorHeight, this.extensionDistance - other.extensionDistance, this.pivotAngle - other.pivotAngle)
    operator fun plus(other: Pose) = Pose(this.elevatorHeight + other.elevatorHeight, this.extensionDistance + other.extensionDistance, this.pivotAngle + other.pivotAngle)
    operator fun times(scalar: Double) = Pose(elevatorHeight * scalar, extensionDistance * scalar, pivotAngle * scalar)

    fun absoluteValue() = Pose(elevatorHeight.absoluteValue(), extensionDistance.absoluteValue(), pivotAngle.absoluteValue())


    companion object {
        val current: Pose
            get() = Pose(Extendavator.elevatorHeight, Extendavator.extensionDistance, Extendavator.pivotAngle)


        val DRIVE = Pose(0.0.inches, 0.0.inches, -90.0.degrees)
        val INTAKE_MIN = Pose(0.0.inches, 30.0.inches, -10.0.degrees)
        val INTAKE_MAX = Pose(0.0.inches, Extendavator.MAX_EXTENSION_DIST, -10.0.degrees)
        val SCORE_BASE = Pose(3.0.feet, 0.0.inches, -10.0.degrees)

        fun generateScorePoseForTower(tower: FieldManager.Tower, distance: Distance = SCORE_BASE.extensionDistance): Pose {
            return Pose(SCORE_BASE.elevatorHeight + 1.3.inches * tower.stackSize.toDouble(), distance, SCORE_BASE.pivotAngle)
        }
    }

}
