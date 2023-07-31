package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.ArmStates;
import frc.robot.Constants.Translations;
import frc.robot.commands.ArmToAngle;
import frc.robot.commands.DriveArmToPosition;
import frc.robot.commands.IntakeToggle;
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

public class CubeAuto extends SequentialCommandGroup {
        public CubeAuto(Arm arm, Intake intake, Wrist wrist) {

                DriveArmToPosition armToHome = new DriveArmToPosition(arm, Constants.ArmConstants.homeAngle, false);

                addCommands(
                        new ParallelDeadlineGroup(new WaitCommand(.3),
                        new IntakeToggle(intake, () -> false),   // SUCK
                       new ArmToAngle(arm, -30)),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(3),
                                        new SetArmState(arm, wrist, ArmStates.HIGH)),
                        new TimedVomit(intake),
                        new InstantCommand(() -> arm.retractPiston()),
                        new ParallelDeadlineGroup(
                                        new WaitCommand(2),
                                        new SetArmState(arm, wrist, ArmStates.HOME))

                // chargeBalance
                );
        }
}