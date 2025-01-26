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
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private TalonSRX algaeIntake, algaePivot;
  private DigitalInput opticalSensor, limitSwitch;
  private PIDController pivotPID;

  private boolean PIDOn = false;
  private double setpoint = 0.0;

  public AlgaeIntakeSubsystem() {
    algaeIntake = new TalonSRX(AlgaeIntakeConstants.INTAKEID);
    algaePivot = new TalonSRX(AlgaeIntakeConstants.PIVOTID);
    algaePivot.setNeutralMode(NeutralMode.Brake);
    opticalSensor = new DigitalInput(AlgaeIntakeConstants.OPTICALID);
    limitSwitch = new DigitalInput(AlgaeIntakeConstants.LSID);
    pivotPID = new PIDController(AlgaeIntakeConstants.KP, AlgaeIntakeConstants.KI, AlgaeIntakeConstants.KD);
    pivotPID.setTolerance(AlgaeIntakeConstants.TOLERANCE);
  }

  public double getEncoder(){
    return -algaeIntake.getSelectedSensorPosition() / 1000;
  }

  public boolean getOpticalValue(){
    return opticalSensor.get();
  }

  public boolean getLMValue(){
    return limitSwitch.get();
  }

  public void runIntakeMotor(double speed){
    algaeIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  double deadzone(double speed){
    if(speed < 0.1 && speed > -0.1){
      return 0;
    }
    else{
      if(speed > AlgaeIntakeConstants.PIVOTMAXSPEED){
        return AlgaeIntakeConstants.PIVOTMAXSPEED;
      }
      else if (speed < -AlgaeIntakeConstants.PIVOTMAXSPEED){
        return -AlgaeIntakeConstants.PIVOTMAXSPEED;
      }
      else{
        return speed;
      }
    }
  }

  public void runPivotMotor(double speed){
    algaePivot.set(TalonSRXControlMode.PercentOutput, deadzone(speed));
  }

  public void stopIntakeMotor(){
    algaeIntake.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void stopPivotMotor(){
    algaePivot.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void enablePID(){
    PIDOn = true;
  }

  public void disablePID(){
    PIDOn = false;
  }

  public boolean isDone(){
    return pivotPID.atSetpoint();
  }

  public void setSetpoint(double newSetpoint){
    setpoint = newSetpoint;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double output = 0.0;

    SmartDashboard.putNumber("[A] Pivot Encoder:", getEncoder());
    SmartDashboard.putBoolean("[A] isFinished?", isDone());
    
    if(PIDOn){
      output = pivotPID.calculate(getEncoder(), setpoint);

      if(output > AlgaeIntakeConstants.INTAKEMAXSPEED){
        output = AlgaeIntakeConstants.INTAKEMAXSPEED;
      }
      else if(output < -AlgaeIntakeConstants.INTAKEMAXSPEED){
        output = -AlgaeIntakeConstants.INTAKEMAXSPEED;
      }
      
      if(isDone() || getLMValue()){
        disablePID();
        stopIntakeMotor();
      }
      else{
        algaePivot.set(TalonSRXControlMode.Current, output);
      }
    }
    
    SmartDashboard.putNumber("[A] Pivot PID Output:", output);
    SmartDashboard.putBoolean("[A] Optical Sensor:", getOpticalValue());
    SmartDashboard.putBoolean("[A] Limit Switch:", getLMValue());

  }
}
