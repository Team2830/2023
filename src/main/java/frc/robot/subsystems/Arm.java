// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.Arm.climberLMotorID);
  private WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.Arm.climberRMotorID);
  private DoubleSolenoid armPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
   Constants.Arm.armForwardID, Constants.Arm.armReverseID);

  /** Creates a new Arm. */
  public Arm() {
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.OpposeMaster);

    configArmMotor(leftMotor, false);
    resetArm();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getArmDegrees());
    // This method will be called once per scheduler run
  }
public double angleToCount(double angle){
   return (angle + 58.76)*(2048*300)/360;
}
  public void resetArm(){
    leftMotor.setSelectedSensorPosition(0);
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);

  }

  public double getArmDegrees() {
    double armSensorValue = leftMotor.getSelectedSensorPosition();
    return armSensorValue*360/(2048*300) - 58.76;
  }

  public void setMotorPosition(double pos){
    double theta = getArmDegrees();
    double thetaRadians = Units.degreesToRadians(theta);
    double ff = Constants.ArmMotorPID.kG * Math.cos(thetaRadians);

    SmartDashboard.putNumber("Feed", ff);

    leftMotor.set(ControlMode.Position, pos, DemandType.ArbitraryFeedForward, ff);
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

  private void configArmMotor(WPI_TalonFX talonFX, boolean inverted){
    talonFX.configFactoryDefault();

    talonFX.setSensorPhase(Constants.ArmMotorPID.kSensorPhase);

    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
     Constants.ArmMotorPID.kPIDLoopIdx,
      Constants.ArmMotorPID.kTimeoutMs);

    
    talonFX.setNeutralMode(NeutralMode.Brake);

    talonFX.setInverted(inverted);

    /*
     * Config the peak and nominal outputs, 12V means full
     */
    talonFX.config_kP(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kP, Constants.ArmMotorPID.kTimeoutMs);
    talonFX.config_kI(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kI, Constants.ArmMotorPID.kTimeoutMs);
    talonFX.config_kD(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kD, Constants.ArmMotorPID.kTimeoutMs);
  }
  
}
