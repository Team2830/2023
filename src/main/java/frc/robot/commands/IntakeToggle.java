// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeToggle extends CommandBase {
  Intake intake;
  private boolean isFinished = false;
  /** Creates a new IntakeVomit. */
  public IntakeToggle(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(intake.getMotorSpeed());
    isFinished = (intake.getMotorSpeed() > IntakeConstants.intakeSpeed - .2);  

    intake.setMotorSpeed(Constants.IntakeConstants.intakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setMotorSpeed(Constants.IntakeConstants.holdSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Current: " + intake.getCurrent());
    return (isFinished || intake.getCurrent() > Constants.IntakeConstants.intakeCurrent);
  }
}
