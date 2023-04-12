// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristManual extends CommandBase {
  private Wrist wrist;
  private DoubleSupplier input;
  /** Creates a new DriveArmToPosition. */
  public WristManual(Wrist wrist, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.input = input;
    addRequirements(wrist);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setMotorSpeed(-input.getAsDouble() * 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
