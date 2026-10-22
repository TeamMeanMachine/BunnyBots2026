package frc.team2471.bunnyBots2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.units.asMeters
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.util.angleTo
import kotlin.math.round


fun aim(targetSupplier: () -> Translation2d = { FieldManager.closestTowerPose }): Command {
    return runCommand(Extendavator, Turret) {
        val relativeTranslation = targetSupplier.invoke().minus(Drive.localizer.pose.translation)
        val angle = relativeTranslation.angle.measure
        val distance = relativeTranslation.norm.meters
        Extendavator.extensionSetpoint = distance
        Turret.fieldCentricSetpoint = angle
        // raise elevator
    }.finallyRun(Extendavator.stow())
}

fun lineUp(targetSupplier: () -> Translation2d = { FieldManager.closestTowerPose }): Command {
    fun getLineUpHeading(): Rotation2d {
        val towerTranslation = targetSupplier()
        val currentTranslation = Drive.localizer.pose.translation
        val angleToTower = currentTranslation.angleTo(towerTranslation).asRotation2d
        val headingError = (angleToTower - Drive.heading).rotations
        return angleToTower - (round(headingError * 4.0) / 4.0).rotations.asRotation2d
    }

    return parallelCommand(
        aim(targetSupplier),
        Drive.joystickDriveAlongCircle(targetSupplier, FieldManager.lineupRadius, ::getLineUpHeading, { Drive.localizer.pose }) // ::getLineUpPose is the same as { getLineUpPose() }
    )
}