// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmStates;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class SetArmState extends CommandBase {
  private static ArmStates lastState = ArmStates.HOME;
  private ArmStates currentState;
  private boolean armFlip;
private double armAngle;
private double wristAngle;


  private Arm arm;
  private Wrist wrist;


  /** Creates a new setArmState. */
  public SetArmState(Arm arm, Wrist wrist, ArmStates state) {
    this.arm = arm;
    this.wrist = wrist;
    currentState = state;

    addRequirements(arm, wrist);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.ArmState = currentState;
    armFlip = arm.getArmSide(currentState) != arm.getArmSide(lastState);

    if(armFlip){
      wrist.setWristAngle(WristConstants.flipAngle);
    }

    switch (currentState){
      case HIGH:
      armAngle = ArmConstants.highAngle;
      wristAngle = WristConstants.highAngle;
      break; 
      case MID:
      armAngle = ArmConstants.midAngle;
      wristAngle = WristConstants.midAngle;
      break;

      case HOME:
      armAngle = ArmConstants.homeAngle;
      wristAngle = WristConstants.homeAngle;
      break;
      case SUBSTATION:
      armAngle = ArmConstants.substationConeAngle;
      wristAngle = WristConstants.substationAngle;
      break;
    }
    armAngle = 0;
    wristAngle = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentState == ArmStates.SUBSTATION){
    //  ArmConstants.substationConeAngle + SmartDashboard.getNumber("Arm adjust", 0);
    }
    arm.setArmAngle(armAngle + SmartDashboard.getNumber("Arm adjust", 0));

    if (armFlip) {
      if (!arm.getArmSide(lastState) && arm.getArmAngle() > 100){
        armFlip = false;
      } else if (arm.getArmSide(lastState) && arm.getArmAngle() < 80) {
        armFlip = false;
      }
    } else {
      wrist.setWristAngle(wristAngle);
    }

    if (currentState == ArmStates.HIGH){
      if (arm.getArmAngle() > 90 && arm.getArmAngle() < 110){
        arm.extendPiston();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      lastState = currentState;
    }else {
      lastState = ArmStates.DEFAULT;
      RobotContainer.ArmState = ArmStates.DEFAULT;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
