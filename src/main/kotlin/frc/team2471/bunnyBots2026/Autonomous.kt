package frc.team2471.bunnyBots2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team2471.bunnyBots2026.tests.*
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.Autonomi
import org.team2471.frc.lib.control.commands.beforeWait
import org.team2471.frc.lib.control.commands.finallyRun
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitUntilCommand
import org.team2471.frc.lib.swerve.sideToSideFlip
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters
import kotlin.math.absoluteValue


object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside Autonomi

    /** Supplier that sets the robot's pose. Used inside [setDrivePositionToAutoStartPose] */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Warmup function for speeding up auto loop times. Runs when selected auto changes. */
    override val warmupFunction: () -> Unit = {
        println("scheduling auto warmup")
        CommandScheduler.getInstance().schedule(warmupDriveAlongPath())
        println("finished scheduling auto warmup")
    }

    /** Chooser for selecting autonomous commands */
    override val autoChooser: LoggedDashboardChooser<AutoCommand?> =
        LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
            addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
            addOption("6x6 Square", AutoCommand(squarePathTest()))
        }

    /** Chooser for test commands */
    override val testChooser: LoggedDashboardChooser<Command?> =
        LoggedDashboardChooser<Command?>("Test Chooser").apply {
            // Set up SysId routines and test command options
            addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
            addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
            addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
            addOption("Set Angle Offsets", Drive.setAngleOffsets())
            addOption("JoystickTest", joystickTest())
            addOption("Drive Slip Current Test", Drive.slipCurrentTest())
            addOption("Drive L/R Static FF Test", Drive.leftRightStaticFFTest())
            addOption("Drive Velocity Volt Test", Drive.velocityVoltTest())
//            addOption("Quest offset Test", Drive.questOffsetTest())
//            addOption("print over hub curves", runOnce{PrintPassOverHubCurves.main(arrayOf(""))})
        }

    /** Autonomous commands */

    private fun eightFootStraight(): Command {
        return Drive.driveAlongChoreoPath(paths["eightFoot"]!!, resetOdometry = true)
    }

    private fun squarePathTest(): Command {
        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
    }

    fun warmupDriveAlongPath(): Command {
        val warmupPath = paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(true)
        return Drive.driveAlongChoreoPath(warmupPath.getSplit(0).get(), exitSupplier = { percent, error -> percent >= 1.0 || Robot.isEnabled}).ignoringDisable(true)
    }
}