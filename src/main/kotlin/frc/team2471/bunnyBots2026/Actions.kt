package frc.team2471.bunnyBots2026

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitUntilCommand
import org.team2471.frc.lib.units.asRotation2d
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.util.angleTo
import kotlin.math.round


fun aim(targetSupplier: () -> FieldManager.Tower = { FieldManager.closestTower }): Command {
    fun getRelative() = targetSupplier.invoke().pose.minus(Drive.localizer.pose.translation)
    return parallelCommand(
        runCommand(Turret) {
            val angle = getRelative().angle.measure
            Turret.goToFieldCentricAngle(angle)
        },
        sequenceCommand(
            waitUntilCommand { getRelative().norm.meters < 4.0.feet },
            Extendavator.animateToPose(Pose.SCORE_BASE),
            runCommand(Extendavator) {
                val distance = getRelative().norm.meters
                val pose = Pose.generateScorePoseForTower(targetSupplier.invoke(), distance)
                Extendavator.goToPose(pose)
            }
        )
    ).finallyRun(Extendavator.stow())
}

fun lineUp(targetSupplier: () -> FieldManager.Tower = { FieldManager.closestTower }): Command {
    fun getLineUpHeading(): Rotation2d {
        val towerTranslation = targetSupplier().pose
        val currentTranslation = Drive.localizer.pose.translation
        val angleToTower = currentTranslation.angleTo(towerTranslation).asRotation2d
        val headingError = (angleToTower - Drive.heading).rotations
        return angleToTower - (round(headingError * 4.0) / 4.0).rotations.asRotation2d
    }

    return parallelCommand(
        aim(targetSupplier),
        Drive.joystickDriveAlongCircle({ targetSupplier().pose }, FieldManager.lineupRadius, ::getLineUpHeading, { Drive.localizer.pose }) // ::getLineUpPose is the same as { getLineUpPose() }
    )
}