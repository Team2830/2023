// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.random.RandomGenerator.ArbitrarilyJumpableGenerator;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private WPI_TalonFX leftMotor = new WPI_TalonFX(0);
  private WPI_TalonFX rightMotor = new WPI_TalonFX(0);
  private DoubleSolenoid armPiston = new DoubleSolenoid(null, 0, 0);

  /** Creates a new Arm. */
  public Arm() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    leftMotor.setInverted(true); // motors turn in opposing directions
    rightMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {

    leftMotor.set(speed);
    rightMotor.set(speed);

  }

  public void extendPiston() {
    armPiston.set(Value.kForward);
  }

  public void retractPiston() {
    armPiston.set(Value.kReverse);
  }

  public void togglePiston(){
    if(armPiston.get() == Value.kForward){ // If the piston is extended...
      retractPiston(); // retract
    } else {
      extendPiston(); // extend
    }
  }
  
}
