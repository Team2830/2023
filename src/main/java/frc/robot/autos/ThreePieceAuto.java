package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.ArmStates;
import frc.robot.Constants.Translations;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveArmToPosition;
import frc.robot.commands.SetArmState;
import frc.robot.commands.TimedVomit;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

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

public class ThreePieceAuto extends SequentialCommandGroup {
        public ThreePieceAuto(Swerve s_Swerve, Wrist wrist, Arm arm, Intake intake) {

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
                Trajectory scoreToFloor1 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Constants.Translations.INSIDE_CONE2, Rotation2d.fromDegrees(180)),
                                                new Pose2d(Constants.Translations.INSIDE_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(158)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Constants.Translations.FLOOR4,
                                                                Rotation2d.fromDegrees(0))),
                                forwardConfig);
                Trajectory floor1ToCubeNode = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Constants.Translations.FLOOR4, Rotation2d.fromDegrees(0)),
                                                new Pose2d(Constants.Translations.INSIDE_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(158)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Constants.Translations.INSIDE_CUBE,
                                                                Rotation2d.fromDegrees(180))),
                                backwardConfig);

                Trajectory scoreToFloor2 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Translations.INSIDE_CUBE, Rotation2d.fromDegrees(180)),
                                                new Pose2d(Translations.INSIDE_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(158)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Translations.INSIDE_CHARGE_CORNER,
                                                                Rotation2d.fromDegrees(0)),

                                                new Pose2d(Translations.FLOOR3,
                                                                Rotation2d.fromDegrees(Translations.FLOOR3_ANGLE))),
                                forwardConfig);
                Trajectory floor2ToRightNode = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Translations.FLOOR3,
                                                Rotation2d.fromDegrees(Translations.FLOOR3_ANGLE)),
                                                new Pose2d(Translations.INSIDE_CHARGE_CORNER,
                                                                Rotation2d.fromDegrees(0)),
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                                new Pose2d(Constants.Translations.INSIDE_COMMUNITY_EXIT,
                                                                Rotation2d.fromDegrees(158)),
                                                new Pose2d(Constants.Translations.INSIDE_CONE1,
                                                                Rotation2d.fromDegrees(180))),
                                backwardConfig);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand pickupFloor4 = new SwerveControllerCommand(
                                scoreToFloor1,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand scoreFloor4 = new SwerveControllerCommand(
                                floor1ToCubeNode,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand pickupFloor3 = new SwerveControllerCommand(
                                scoreToFloor2,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                SwerveControllerCommand scoreFloor3 = new SwerveControllerCommand(
                                floor2ToRightNode,
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

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(scoreToFloor1.getInitialPose())),
                                new InstantCommand(() -> s_Swerve.zeroGyro()),
                                new InstantCommand(() -> arm.resetArm()),
                                new InstantCommand(() -> wrist.resetWrist()),
                                new TimedVomit(intake),
                                pickupFloor4


                // chargeBalance

                );
        }
}