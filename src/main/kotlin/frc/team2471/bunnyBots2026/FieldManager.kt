package frc.team2471.bunnyBots2026

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Filesystem
import frc.team2471.bunnyBots2026.FieldManager.reflectAcrossField
import frc.team2471.bunnyBots2026.FieldManager.rotateAroundField
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.isRedAlliance

object FieldManager {
    private val table = NetworkTableInstance.getDefault().getTable("FieldManager")

    val aprilTagFieldLayout: AprilTagFieldLayout = AprilTagFieldLayout(Filesystem.getDeployDirectory().path + "/bunnybotsTags.json")
    val allAprilTags = aprilTagFieldLayout.tags

    // x
    val fieldWidth = aprilTagFieldLayout.fieldWidth.meters
    // y
    val fieldLength = aprilTagFieldLayout.fieldLength.meters

    val fieldDimensions = UTranslation2d(fieldLength, fieldWidth)

    val fieldHalfWidth = fieldWidth / 2.0
    val fieldHalfLength = fieldLength / 2.0

    val fieldCenter = fieldDimensions / 2.0

    val towerTagToCenterTransform = Transform3d(allAprilTags.getID(3).pose.y - allAprilTags.getID(2).pose.y, 0.0, 0.0, Rotation3d.kZero)
    val allTowerPositions: List<Translation2d> = List(8) {
        allAprilTags.getID(it + 1).pose.transformBy(towerTagToCenterTransform).translation.toTranslation2d()
    }
    val allTowers: Map<Int, Tower> =
        allTowerPositions.mapIndexed { index, pose -> Tower(index, pose) }.associateBy { it.id }

    val lineupRadius = 3.0.feet

    @get:AutoLogOutput(key = "FieldManager/closestTowerPose")
    val closestTowerPose get() = getClosestTowerPose(Drive.localizer.pose)

    init {
    }

    fun lateInit() {
        Logger.recordOutput("FieldManager/allTags", *allAprilTags.map { it.pose }.toTypedArray())
        Logger.recordOutput("FieldManager/towerTagToCenterTransform", towerTagToCenterTransform)
        Logger.recordOutput("FieldManager/allTowers", *allTowerPositions.toTypedArray())
    }


    fun getClosestTower(pose: Pose2d): Tower {
        return allTowers.values.minBy { it.pose.getDistance(pose.translation) }
    }
    fun getClosestTowerPose(pose: Pose2d): Translation2d {
        return getClosestTower(pose).pose
    }



    data class Tower(val id: Int, val pose: Translation2d, var stackSize: Int = 0)

    /**
     * Extends an apriltag list and searches for the tag with the given id.
     */
    fun List<AprilTag>.getID(id: Int): AprilTag {
        return this.find { it.ID == id } ?: throw IllegalArgumentException("Tag ID $id not found")
    }

    /**
     * Reflects [Translation2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Translation2d.rotateAroundField
     */
    fun Translation2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Translation2d {
        return if (doReflect()) Translation2d(fieldLength.asMeters - x, y) else this
    }

    /**
     * Reflects [Pose2d] across the midline of the field. Useful for mirrored field layouts (2023, 2024).
     * Units must be meters
     * @param doReflect Supplier to perform reflection. Default: true
     * @see Pose2d.rotateAroundField
     */
    fun Pose2d.reflectAcrossField(doReflect: () -> Boolean = { true }): Pose2d {
        return if (doReflect()) Pose2d(fieldLength.asMeters - x, y, (rotation - 180.0.degrees.asRotation2d).wrap()) else this
    }

    /**
     * Rotates the [Translation2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Translation2d.reflectAcrossField
     */
    fun Translation2d.rotateAroundField(doRotate: () -> Boolean = { true }): Translation2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Rotates the [Pose2d] 180 degrees around the center of the field. Useful for reflected field layouts (2022, 2025).
     * Units must be meters
     * @param doRotate Supplier to perform rotation. Default: true
     * @see Pose2d.reflectAcrossField
     */
    fun Pose2d.rotateAroundField(doRotate: () -> Boolean = { true }): Pose2d {
        return if (doRotate()) this.rotateAround(fieldCenter, 180.0.degrees.asRotation2d) else this
    }

    /**
     * Returns if the [Translation2d] is on the red alliance side of the field.
     */
    fun Translation2d.onRedSide(): Boolean = this.x > fieldCenter.x.asMeters
    /**
     * Returns if the [Translation2d] is on the blue alliance side of the field.
     */
    fun Translation2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Translation2d] is closer to your current alliance's side of the field.
     */
    fun Translation2d.onFriendlyAllianceSide() = this.onRedSide() == isRedAlliance
    /**
     * Returns if the [Translation2d] is closer to your opponent alliance's side of the field.
     */
    fun Translation2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()

    /**
     * Returns if the [Pose2d] is on the red alliance side of the field.
     */
    fun Pose2d.onRedSide(): Boolean = this.translation.onRedSide()
    /**
     * Returns if the [Pose2d] is on the blue alliance side of the field.
     */
    fun Pose2d.onBlueSide(): Boolean = !this.onRedSide()
    /**
     * Returns if the [Pose2d] is closer to your current alliance's side of the field.
     */
    fun Pose2d.onFriendlyAllianceSide() = this.translation.onFriendlyAllianceSide()
    /**
     * Returns if the [Pose2d] is closer to your opponent alliance's side of the field.
     */
    fun Pose2d.onOpposingAllianceSide() = !this.onFriendlyAllianceSide()
}
