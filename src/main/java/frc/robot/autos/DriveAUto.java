package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.Translations;
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

public class DriveAUto extends SequentialCommandGroup {
        public DriveAUto(Swerve s_Swerve) {

                /*
                 * 
                 * Drive Commands
                 * 
                 */

                TrajectoryConfig forwardConfig = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond/3.5,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics)
                                .setReversed(true);

                TrajectoryConfig backwardConfig = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond/3.5,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory driveOnCharge = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                List.of(new Pose2d(Constants.Translations.OUTSIDE_CONE1, Rotation2d.fromDegrees(0)),
                                new Pose2d(new Translation2d(-Units.inchesToMeters(10),Units.inchesToMeters(22)), Rotation2d.fromDegrees(0)),
                                                new Pose2d(Constants.Translations.OUTSIDE_CONE2, Rotation2d.fromDegrees(0))
                                                // Interior Waypoints
                                                // End 3 meters straight ahead of where we started, facing forward
                                              ),
                                forwardConfig);
                

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand passCharge = new SwerveControllerCommand(
                                driveOnCharge,
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

               

                AutoBalance chargeBalance = new AutoBalance(s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.resetOdometry(driveOnCharge.getInitialPose())),
                                new InstantCommand(() -> s_Swerve.zeroGyro()),
                                passCharge
                                
                                

                // chargeBalance

                );
        }
}