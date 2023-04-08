// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class WristToAngle extends CommandBase {
  private Wrist wrist;
  private double angle;
  private double goalAngle;

  PIDController controller = new PIDController(Constants.ArmMotorPID.kP,
  Constants.ArmMotorPID.kI,
  Constants.ArmMotorPID.kD);

  /** Creates a new DriveArmToPosition. */
  public WristToAngle(Wrist wrist, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.angle = angle;
    addRequirements(wrist);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalAngle = angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      goalAngle = angle + SmartDashboard.getNumber("Arm adjust", 0);
    

    wrist.setWristAngle(goalAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
