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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristMotorPID;

public class Wrist extends SubsystemBase {
  private WPI_TalonFX wristMotor = new WPI_TalonFX(Constants.WristConstants.wristID);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(9);
  private double angleOffset = 0;
  
  
  PIDController wristController = new PIDController(Constants.WristMotorPID.kP,
  Constants.WristMotorPID.kI,
  Constants.WristMotorPID.kD);
  

  /** Creates a new Arm. */
  public Wrist() {
    encoder.setDistancePerRotation(360.0);
    encoder.setPositionOffset(.18);
    wristMotor.setNeutralMode(NeutralMode.Brake);
    configWristMotor(wristMotor, true);
    resetWrist();
    if(Math.abs(encoder.getDistance()) > 180){
      angleOffset = 360;
    } else {
      angleOffset = 0;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist Absolute", encoder.getAbsolutePosition());
    // This method will be called once per scheduler run
  }

  public double angleToCount(double angle) {
    return (angle + Constants.ArmConstants.startAngle) * (2048 * 125) / 360;
  }

  public double getWristAngle() {
    //double wristPos = wristMotor.getSelectedSensorPosition();
    //return wristPos * 360 / (2048 * 125) + Constants.WristConstants.startAngle;
    return (encoder.getDistance() - angleOffset);
  }

  public void resetWrist() {
    wristMotor.setSelectedSensorPosition(0); //TODO Use absolute encoder
  }

  public void setMotorSpeed(double speed) {
    wristMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, Math.signum(speed) * WristMotorPID.kG); 
  }

  
  // TODO: Add a condition to switch to manual if the encoder is disconnected
  public void setWristAngle(double angle){
    setMotorSpeed(wristController.calculate(getWristAngle(), angle));
  }

 
  

  private void configWristMotor(WPI_TalonFX talonFX, boolean inverted) {
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
