// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  // Motors
  private TalonFX elevatorMotor;

  private CurrentLimitsConfigs currentLimits;

  // Digital Inputs
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  // pid variables
  private PIDController pid;
  private double setpoint;
  private double previousError;
  private double currentError;
  private boolean pidOn;

  // outputs
  private double output;
  private double manualOutput;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(ElevatorConstants.LIFTID);
    topLimitSwitch = new DigitalInput(ElevatorConstants.UPPERLSID);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.BOTTOMLSID);
    pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);

    elevatorMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    output = 0;
    manualOutput = 0;

    pid.setTolerance(ElevatorConstants.TOLERANCE);
    setpoint = getEncoder();
    previousError = 0;

    pidOn = false;

    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setElevatorSpeed(double speed) {
    manualOutput = speed;
  }

  public boolean getTopLimitSwitch() {
    return !topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch() {
    return !bottomLimitSwitch.get();
  }

  public void turnPIDOn() {
    pidOn = true;
  }

  public void turnPIDOff() {
    pidOn = false;
  }

  public boolean getPID() {
    return pidOn;
  }

  public double getEncoder() {
    return elevatorMotor.getPosition().getValueAsDouble();
  }

  public void resetEncoder() {
    elevatorMotor.setPosition(0);
  }

  public void stopElevator() {
    output = 0;
    elevatorMotor.set(0);
  }

  // deadzones manual inputs from -0.1 to 0.1 and caps the output at
  // ElevatorConstants.MAXSPEED
  private double deadzone(double input) {
    if (Math.abs(input) < 0.1) {
      return 0;
    } else if (input > ElevatorConstants.MAXSPEED) {
      return ElevatorConstants.MAXSPEED;
    } else if (input < -ElevatorConstants.MAXSPEED) {
      return -ElevatorConstants.MAXSPEED;
    } else {
      return input;
    }
  }

  public void setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
    resetI();
  }

  // resets the I value on overshoot
  private void resetI() {
    currentError = pid.getPositionError();
    if (currentError > 0 && previousError < 0) {
      pid.reset();
    } else if (currentError < 0 && previousError > 0) {
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

    if (getBottomLimitSwitch()) {
      resetEncoder();
    }

    resetI();

    // If pid is on, runs pid code to set output
    if (pidOn) {
      output = pid.calculate(getEncoder(), setpoint);

      // Caps output at ElevatorConstants.MAXSPEED
      if (output > ElevatorConstants.MAXSPEED) {
        output = ElevatorConstants.MAXSPEED;
      } else if (output < -ElevatorConstants.MAXSPEED) {
        output = -ElevatorConstants.MAXSPEED;
      }
    }

    // If pid is off, runs manual control (TESTING CODE ONLY)
    else{
    output = deadzone(manualOutput);
    }

    // Stops elevator if it hits bottom limit switch and is moving in the same direction
    if (getBottomLimitSwitch() && output < 0) {
      output = 0;
      setpoint = 0;
    }
    // Sets setpoint to current encoder value if elevator hits top limit switch
    else if (getTopLimitSwitch() && output > 0){
      setpoint = getEncoder();
    }

    // Final call to set output
    elevatorMotor.set(output);

    // SmartDashboard
    SmartDashboard.putNumber("[E] Enc", getEncoder());
    SmartDashboard.putNumber("[E] Output", output);
    SmartDashboard.putNumber("[E] Manual Output", manualOutput);
    SmartDashboard.putNumber("[E] Setpoint", setpoint);
    SmartDashboard.putBoolean("[E] isAtSetpoint", atSetpoint());
    SmartDashboard.putBoolean("[E] Top LS", getTopLimitSwitch());
    SmartDashboard.putBoolean("[E] Bottom LS", getBottomLimitSwitch());
    SmartDashboard.putBoolean("[E] pidON", pidOn);
    SmartDashboard.putData("[E] Elevator pid", pid);
    
    SmartDashboard.putNumber("[E] Stator Current", elevatorMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("[E] Supply Current", elevatorMotor.getSupplyCurrent().getValueAsDouble());
  }
}
