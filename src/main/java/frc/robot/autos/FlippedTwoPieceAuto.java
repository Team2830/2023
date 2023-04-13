package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.FlippedTranslations;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveArmToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FlippedTwoPieceAuto extends SequentialCommandGroup {
        public FlippedTwoPieceAuto(Swerve s_Swerve, Arm arm, Intake intake) {

                /*
                 * 
                 * Drive Commands
                 * 
                 */

                TrajectoryConfig forwardConfig = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics)
                                .setReversed(true);

                TrajectoryConfig backwardConfig = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory pickupTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Constants.FlippedTranslations.INSIDE_CONE1,
                                                Rotation2d.fromDegrees(0)),
                                                new Pose2d(Constants.FlippedTranslations.OLD_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(0)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Constants.FlippedTranslations.OLD_FLOOR4,
                                                                Rotation2d.fromDegrees(0))),
                                forwardConfig);
                Trajectory scoreCube = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Constants.FlippedTranslations.OLD_FLOOR4, Rotation2d.fromDegrees(0)),
                                                new Pose2d(Constants.FlippedTranslations.OLD_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(0)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Constants.FlippedTranslations.OLD_INSIDE_CUBE,
                                                                Rotation2d.fromDegrees(0))),
                                backwardConfig);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand pickupFloor4 = new SwerveControllerCommand(
                                pickupTrajectory,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand scoreFloor4 = new SwerveControllerCommand(
                                scoreCube,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                /*
                 * Intake Commands
                 */

                InstantCommand intakeVomit = new InstantCommand(() -> intake.intakeVomit(), intake);

                /*
                 * Arm Commands
                 */

                DriveArmToPosition armToHigh = new DriveArmToPosition(arm, Constants.ArmConstants.highAngle, true);

                DriveArmToPosition armToHome = new DriveArmToPosition(arm, Constants.ArmConstants.homeAngle, false);

                InstantCommand extendArm = new InstantCommand(() -> arm.extendPiston(), arm);

                InstantCommand retractArm = new InstantCommand(() -> arm.retractPiston(), arm);

                AutoBalance chargeBalance = new AutoBalance(s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(pickupTrajectory.getInitialPose())),
                                new InstantCommand(() -> s_Swerve.zeroGyro(0)),
                                new InstantCommand(() -> arm.resetArm()),
                                new DriveArmToPosition(arm, Constants.ArmConstants.highAngle, true),
                                new WaitCommand(1.5),
                                new InstantCommand(() -> intake.intakeVomit(), intake),
                                new WaitCommand(.25),
                                new InstantCommand(() -> arm.retractPiston(), arm),
                                new ParallelDeadlineGroup(
                                                pickupFloor4,
                                                new SequentialCommandGroup(
                                                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                                                armToHome),
                                                                extendArm)),
                                new ParallelDeadlineGroup(new WaitCommand(.75),
                                                new InstantCommand(() -> intake.intakeGrab(), intake)),
                                new ParallelDeadlineGroup(scoreFloor4,
                                                new SequentialCommandGroup(retractArm, new WaitCommand(.25),
                                                                new DriveArmToPosition(arm, 165, true))),

                                new InstantCommand(() -> intake.intakeVomit(), intake), new WaitCommand(.25)

                // chargeBalance

                );
        }
}