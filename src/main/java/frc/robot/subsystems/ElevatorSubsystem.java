// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorMotor;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  private PIDController pid;
  private double setpoint;
  private double previousError;
  private double currentError;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(ElevatorConstants.LIFTID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.UPPERLSID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOMLSID);
    pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);

    pid.setTolerance(ElevatorConstants.TOLERANCE);
    setpoint = getEncoder();
    previousError = 0;

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void moveElevator(double speed) {
    elevatorMotor.set(deadzone(speed));
  }

  public boolean getTopLimitSwitch() {
    return topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch() {
    return bottomLimitSwitch.get();
  }

  public double getEncoder(){
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder(){
    elevatorMotor.setPosition(0);
  }

  public void stopElevator() {
    elevatorMotor.set(0);
  }

  public double deadzone(double input) {
    if (Math.abs(input) < 0.1) {
      return 0;
    }
    else if (input > ElevatorConstants.MAXSPEED) {
      return ElevatorConstants.MAXSPEED;
    }
    else if (input < -ElevatorConstants.MAXSPEED) {
      return -ElevatorConstants.MAXSPEED;
    }
    else {
      return input;
    }
  }

  public void NewSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  public void resetI() {
    currentError = pid.getPositionError();

    if (currentError > 0 && previousError < 0) {
      pid.reset();
    } 
    else if (currentError < 0 && previousError > 0) {
      pid.reset();
    }

    previousError = currentError;
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getBottomLimitSwitch()){
      resetEncoder();
    }

    resetI();

    double output = pid.calculate(getEncoder(), setpoint);

    if (getBottomLimitSwitch() && output < 0){
      stopElevator();
      output = 0;
    }

    else if (getTopLimitSwitch() && output > 0){
      stopElevator();
      output = 0;
    }

    else if (atSetpoint()){
      output = 0;
    }

    else if (output > ElevatorConstants.MAXSPEED) {
      elevatorMotor.set(ElevatorConstants.MAXSPEED);
    } 
    else if (output < -ElevatorConstants.MAXSPEED) {
      elevatorMotor.set(-ElevatorConstants.MAXSPEED);
    }
    else {
      elevatorMotor.set(output);
    }

    // SmartDashboard
    SmartDashboard.putNumber("[E] Enc", getEncoder());
    SmartDashboard.putNumber("[E] Output", output);
    SmartDashboard.putNumber("[E] Setpoint", setpoint);
    SmartDashboard.putBoolean("[E] isAtSetpoint", atSetpoint());
    SmartDashboard.putBoolean("[E] Top LS", getTopLimitSwitch());
    SmartDashboard.putBoolean("[E] Bottom LS", getBottomLimitSwitch());
  }
}
