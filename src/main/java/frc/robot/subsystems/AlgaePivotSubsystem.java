// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaePivotConstants;

public class AlgaePivotSubsystem extends SubsystemBase {
  private TalonSRX algaePivot;
  private DigitalInput limitSwitch;

  private PIDController pivotPID;
  private boolean PIDOn = false;
  private double output = 0.0;
  private double setpoint = 0;

  public AlgaePivotSubsystem() {
    algaePivot = new TalonSRX(AlgaePivotConstants.PIVOTID);

    limitSwitch = new DigitalInput(AlgaePivotConstants.LSID);

    pivotPID = new PIDController(AlgaePivotConstants.KP, AlgaePivotConstants.KI, AlgaePivotConstants.KD);
    pivotPID.setTolerance(AlgaePivotConstants.TOLERANCE);

    algaePivot.setNeutralMode(NeutralMode.Brake);
    algaePivot.enableCurrentLimit(AlgaePivotConstants.CURRENTLIMIT);
    algaePivot.configPeakCurrentLimit(35);
    algaePivot.configPeakCurrentDuration(100, 50);
    algaePivot.configContinuousCurrentLimit(30);
  }

  //returns the raw integrated encoder value of the algae pivot motor 
  public double getEncoder() {
    return algaePivot.getSensorCollection().getQuadraturePosition();
  }

  //resets the algae pivot integrated encoder to 0
  void resetEncoder(){
    algaePivot.getSensorCollection().setQuadraturePosition(0, 0);
  }

  //returns the value of the limit switch (true or false)
  public boolean getLSValue() {
    return limitSwitch.get();
  }

   //stops the algae pivot motor
  public void stopPivotMotor() {
    algaePivot.set(TalonSRXControlMode.PercentOutput, 0);
  }

  //turns the PID on
  public void enablePID() {
    PIDOn = true;
  }

  //turns the PID off
  public void disablePID() {
    PIDOn = false;
  }

  //returns if the PID is finished (PID is at setpoint or not)
  public boolean isDone() {
    return pivotPID.atSetpoint();
  }

  //sets the new setpoint for the PID
  public void setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  //sets the new output of the algae pivot motor (manual control) and checks the output to see if it's within the max speed parameters
  public void setOutput(double newOutput){
    output = newOutput;
    if(output > AlgaePivotConstants.PIVOTMAXSPEED){
      output = AlgaePivotConstants.PIVOTMAXSPEED;
    }
    else if(newOutput < -AlgaePivotConstants.PIVOTMAXSPEED){
      output = -AlgaePivotConstants.PIVOTMAXSPEED;
    }
  }

  @Override
  public void periodic() {
     //prints the raw integrated encoder value of the pivot motor, if the PID is finished or not
    SmartDashboard.putNumber("[A] Pivot Encoder:", getEncoder());
    SmartDashboard.putBoolean("[A] PidOn?", PIDOn);
    SmartDashboard.putBoolean("[A] isFinished?", isDone());
    SmartDashboard.putData("[A] PID Controller", pivotPID);
    // Allah uh akbar

    //reset the algae pivot encoders if touching the limit switch
    // if(getLSValue()){
    //   resetEncoder();
    // }

    //if PID is on, calculate the output of the pivot motor
    if (PIDOn) {
      output = pivotPID.calculate(getEncoder(), setpoint);
      
      //if the PID is at the setpoint, disable the PID and stop the pivot motor
      if (isDone()) {
        disablePID();
        stopPivotMotor();
      }
    } 

    //if the pivot motor is touching the limit switch and the output is negative, stop the motor
    if(getLSValue() && output < 0){
      stopPivotMotor();
    }  

    //checks the output of the pivot motor and sets it to the motor if it is within the deadzone and max speeds
    algaePivot.set(TalonSRXControlMode.PercentOutput, output);

    //prints the output of the pivot motor and if the limit switch is pressed or not
    SmartDashboard.putNumber("[A] Pivot PID Output:", output);
    SmartDashboard.putBoolean("[A] Limit Switch:", getLSValue());
  }
}
