package frc.team2471.bunnyBots2026

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.units.*
import org.team2471.frc.lib.util.isRedAlliance
import kotlin.math.cos
import kotlin.math.sin

object FieldManager: SubsystemBase() {
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


    private val wristComponentOffset = Translation3d(0.31, 0.0, 0.445)
                        //    Turret, Carriage, Extension, Intake
    @get:AutoLogOutput(key = "Components/measuredComponents")
    val measuredComponents: Array<Pose3d>
        get() {
            return createComponentArray(
                Turret.robotCentricPosition,
                Extendavator.motorHeight,
                Extendavator.motorExtension,
                Intake.wristMotor.position.value
            )
        }

    @get:AutoLogOutput(key = "Components/setpointComponents")
    val setpointComponents: Array<Pose3d>
        get() {
            return createComponentArray(
                Turret.robotCentricSetpoint,
                Extendavator.heightSetpoint,
                Extendavator.extensionSetpoint,
                Intake.wristMotorSetpoint
            )
        }
    val timer = Timer().apply { start() }

    init {
    }

    fun lateInit() {
        Logger.recordOutput("FieldManager/allTags", *allAprilTags.map { it.pose }.toTypedArray())
        Logger.recordOutput("FieldManager/towerTagToCenterTransform", towerTagToCenterTransform)
        Logger.recordOutput("FieldManager/allTowers", *allTowerPositions.toTypedArray())
        Logger.recordOutput("Components/Zero", *arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d()))
    }

    override fun periodic() {
        val t = timer.get()
        val turretAngle = t / 10.0
        val turretRotation = Rotation3d(0.0, 0.0, turretAngle)
        val carriageHeight = (t/5.0).mod(1.5)
        val extensionDistance = (t/5.0).mod(0.33)
        val extensionTranslation = Translation3d(extensionDistance * cos(turretAngle), extensionDistance * sin(turretAngle), carriageHeight)
        val wristExtensionDistance = extensionDistance + wristComponentOffset.x
        val wristTranslation = Translation3d(wristExtensionDistance * cos(turretAngle), wristExtensionDistance * sin(turretAngle), carriageHeight + wristComponentOffset.z)


        val components = arrayOf(
            Pose3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, turretAngle)),
            Pose3d(Translation3d(0.0, 0.0, carriageHeight), Rotation3d(0.0, 0.0, turretAngle)),
            Pose3d(extensionTranslation, Rotation3d(0.0, 0.0, turretAngle)),
            Pose3d(wristTranslation, Rotation3d(0.0, t*2.0, turretAngle)))
        Logger.recordOutput("Components/Test", *components)
    }


    fun getClosestTower(pose: Pose2d): Tower {
        return allTowers.values.minBy { it.pose.getDistance(pose.translation) }
    }
    fun getClosestTowerPose(pose: Pose2d): Translation2d {
        return getClosestTower(pose).pose
    }


    fun createComponentArray(turretAngle: Angle, carriageHeight: Distance, extensionDistance: Distance, wristAngle: Angle): Array<Pose3d> {
        val turretRotation = Rotation3d(0.0.degrees, 0.0.degrees, turretAngle)
        val wristRotation = Rotation3d(0.0.degrees, wristAngle, turretAngle)

        val extensionTranslation = Translation3d(extensionDistance * cos(turretAngle), extensionDistance * sin(turretAngle), carriageHeight)
        val wristExtensionDistance = extensionDistance + wristComponentOffset.x.meters
        val wristTranslation = Translation3d(wristExtensionDistance * cos(turretAngle), wristExtensionDistance * sin(turretAngle), carriageHeight + wristComponentOffset.z.meters)

        return arrayOf(
            Pose3d(Translation3d(), turretRotation),
            Pose3d(Translation3d(0.0, 0.0, carriageHeight.asMeters), turretRotation),
            Pose3d(extensionTranslation, turretRotation),
            Pose3d(wristTranslation, wristRotation)
        )
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
