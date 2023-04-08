package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.Translations;
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

public class CubeAuto extends SequentialCommandGroup {
        public CubeAuto(Arm arm, Intake intake) {

                DriveArmToPosition armToHome = new DriveArmToPosition(arm, Constants.ArmConstants.homeAngle, false);

                addCommands(
                                new InstantCommand(() -> arm.resetArm()),
                                new DriveArmToPosition(arm, Constants.ArmConstants.cubeHigh, true),
                                new WaitCommand(1.5),
                                new InstantCommand(() -> intake.intakeVomit(), intake),
                                new WaitCommand(1),
                                new InstantCommand(() -> arm.retractPiston(), arm),
                                armToHome
                );
        }
}