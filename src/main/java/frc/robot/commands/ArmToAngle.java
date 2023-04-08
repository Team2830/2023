// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmStates;
import frc.robot.subsystems.Arm;

public class ArmToAngle extends CommandBase {
  private Arm arm;
  private double angle;
  private double calcAngle;

  PIDController controller = new PIDController(Constants.ArmMotorPID.kP,
  Constants.ArmMotorPID.kI,
  Constants.ArmMotorPID.kD);

  /** Creates a new DriveArmToPosition. */
  public ArmToAngle(Arm arm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.angle = angle;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calcAngle = angle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.ArmState == ArmStates.SUBSTATION){
      calcAngle = Constants.ArmConstants.substationConeAngle + SmartDashboard.getNumber("Arm adjust", 0);
    }

    if(RobotContainer.ArmState == ArmStates.HIGH){
      if(arm.getArmAngle() > 90 && arm.getArmAngle() < 120){
        arm.extendPiston();
      }
    }
    arm.setArmAngle(calcAngle);
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
