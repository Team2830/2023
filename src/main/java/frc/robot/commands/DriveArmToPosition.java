// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class DriveArmToPosition extends CommandBase {
  private Arm arm;
  private double angle;
  private boolean extend;

  /** Creates a new DriveArmToPosition. */
  public DriveArmToPosition(Arm arm, double angle, boolean extend ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.angle = angle;
    this.extend = extend;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setMotorPosition(arm.angleToCount(angle));
    if (extend && arm.getArmAngle() > 80){
      arm.extendPiston();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double angleCurrent = arm.getArmAngle();
  return (angleCurrent > angle - Constants.ArmConstants.angleError
  && angleCurrent < angle + Constants.ArmConstants.angleError);
  }
}
