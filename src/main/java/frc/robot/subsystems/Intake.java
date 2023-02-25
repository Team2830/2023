// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
   Constants.Intake.armForwardID, Constants.Intake.armReverseID);


  /** Creates a new Intake. */

  public Intake() {
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void extendPiston() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retractPiston() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void togglePiston(){
    if(intakeSolenoid.get() == Value.kForward){ // If the piston is extended...
      retractPiston(); // retract
      System.out.println("RETRACT");
    } else {
      extendPiston(); // 
      System.out.println("EXTEND");
    }
  }
}
