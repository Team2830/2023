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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmStates;

public class Arm extends SubsystemBase {
  private WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.ArmConstants.climberLMotorID);
  private WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.ArmConstants.climberRMotorID);
  
  private DoubleSolenoid armPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.ArmConstants.armForwardID, Constants.ArmConstants.armReverseID);

  private DutyCycleEncoder encoder = new DutyCycleEncoder(0);

  PIDController controller = new PIDController(Constants.ArmMotorPID.kP,
  Constants.ArmMotorPID.kI,
  Constants.ArmMotorPID.kD);

  private double angleOffset = 0;

  /** Creates a new Arm. */
  public Arm() {
    encoder.setDistancePerRotation(360);
    
    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.OpposeMaster);

    // leftMotor.configReverseSoftLimitThreshold(angleToCount(Constants.Arm.startAngle + 3));
    // leftMotor.configReverseSoftLimitEnable(true);
    // rightMotor.configReverseSoftLimitThreshold(angleToCount(Constants.Arm.startAngle + 3));
    // rightMotor.configReverseSoftLimitEnable(true);

    configArmMotor(leftMotor, false);
    resetArm();
    if(Math.abs(encoder.getDistance()) > 180){
      angleOffset = 360;
    } else {
      angleOffset = 0;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", getArmAngle());
    // This method will be called once per scheduler run
  }

  public double angleToCount(double angle) {
    return (angle + 58.76) * (2048 * 300) / 360;
  }

  public double getArmAngle() {
    //double armSensorValue = leftMotor.getSelectedSensorPosition();
    //return armSensorValue * 360 / (2048 * 300) + Constants.ArmConstants.startAngle;
    return -encoder.getDistance() + 61 - angleOffset;
  }

  public void setArmAngle(double angle){
    setMotorSpeed(controller.calculate(getArmAngle(), angle));
  }

  public void resetArm() {
    leftMotor.setSelectedSensorPosition(0);
  }

  public void setMotorSpeed(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, getFF());
  }

  public void setMotorPosition(double pos) {

    SmartDashboard.putNumber("Feed", getFF());

    leftMotor.set(ControlMode.Position, pos, DemandType.ArbitraryFeedForward, getFF());
  }

  public double getFF(){
    double theta = getArmAngle();
    double thetaRadians = Units.degreesToRadians(theta);
    return Constants.ArmMotorPID.kG * Math.cos(thetaRadians);
  }



  public void extendPiston() {
    armPiston.set(Value.kForward);
  }

  public void retractPiston() {
    armPiston.set(Value.kReverse);
  }

  public void togglePiston() {
    if (armPiston.get() == Value.kForward) { // If the piston is extended...
      retractPiston(); // retract
    } else {
      extendPiston(); // extend
    }
  }

  /*
   * Determines which side of the arm any armState is on
   * false if battery side, true if arm post side
   */
  public boolean getArmSide(ArmStates ArmState){
    boolean returnVal = false;
    switch (ArmState){
      case HIGH:
      case MID:
      returnVal = true;
      break;

      case HOME:
      case CONE:
      case CUBE:
      returnVal = false;
      break;
      
      case DEFAULT:
      returnVal = getArmAngle() > 90 ? true : false;
      break;
    }
    return returnVal;
  }

  private void configArmMotor(WPI_TalonFX talonFX, boolean inverted) {
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
