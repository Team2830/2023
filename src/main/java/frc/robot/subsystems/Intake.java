// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.IntakeConstants.intakeID);
  public Intake() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    configWristMotor(intakeMotor, true);
    // openIntake();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake current", getCurrent());
    
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed){
    intakeMotor.set(speed);
    SmartDashboard.putNumber("Intake Motor Speed", speed);
  }
  
  public void intakeGrab(){
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }
  public void intakeVomit(){
    intakeMotor.set(Constants.IntakeConstants.vomitSpeed);
  }
  public double getCurrent(){
    return intakeMotor.getStatorCurrent();
  }
public double getMotorSpeed (){
  return intakeMotor.getMotorOutputPercent();
}
  private void configWristMotor(WPI_TalonFX talonFX, boolean inverted) {
    talonFX.configFactoryDefault();

    talonFX.setSensorPhase(Constants.ArmMotorPID.kSensorPhase);

    talonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.ArmMotorPID.kPIDLoopIdx,
        Constants.ArmMotorPID.kTimeoutMs);

    talonFX.setInverted(inverted);

    /*
     * Config the peak and nominal outputs, 12V means full
     */
    talonFX.config_kP(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kP, Constants.ArmMotorPID.kTimeoutMs);
    talonFX.config_kI(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kI, Constants.ArmMotorPID.kTimeoutMs);
    talonFX.config_kD(Constants.ArmMotorPID.kPIDLoopIdx, Constants.ArmMotorPID.kD, Constants.ArmMotorPID.kTimeoutMs);
  }
}