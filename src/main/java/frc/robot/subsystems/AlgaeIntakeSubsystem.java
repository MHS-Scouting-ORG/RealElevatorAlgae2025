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
  private TalonSRX algaeIntake, algaePivot;
  private DigitalInput opticalSensor, limitSwitch;
  private PIDController pivotPID;

  private boolean PIDOn = false;
  private double output = 0.0;
  private double setpoint = 0;

  public AlgaeIntakeSubsystem() {
    algaeIntake = new TalonSRX(AlgaeIntakeConstants.INTAKEID);
    algaePivot = new TalonSRX(AlgaeIntakeConstants.PIVOTID);

    opticalSensor = new DigitalInput(AlgaeIntakeConstants.OPTICALID);
    limitSwitch = new DigitalInput(AlgaeIntakeConstants.LSID);

    pivotPID = new PIDController(AlgaeIntakeConstants.KP, AlgaeIntakeConstants.KI, AlgaeIntakeConstants.KD);
    pivotPID.setTolerance(AlgaeIntakeConstants.TOLERANCE);

    algaePivot.setNeutralMode(NeutralMode.Brake);
    algaePivot.configPeakCurrentLimit(35, 10);
    algaePivot.configPeakCurrentDuration(200, 10);
    algaePivot.configContinuousCurrentLimit(30, 10);

    algaeIntake.setNeutralMode(NeutralMode.Brake);
    algaeIntake.configPeakCurrentLimit(35, 10);
    algaeIntake.configPeakCurrentDuration(200, 10);
    algaeIntake.configContinuousCurrentLimit(30, 10);
    }

  //returns the raw integrated encoder value of the algae pivot motor 
  public double getEncoder() {
    return algaePivot.getSelectedSensorPosition();
  }

  //returns the value of the optical sensor (true or false)
  public boolean getOpticalValue() {
    return !opticalSensor.get();
  }

  //returns the value of the limit switch (true or false)
  public boolean getLSValue() {
    return limitSwitch.get();
  }

  //checks if the speed of the pivot motor is within the deadzone and max speed parameters
  double deadzone(double newOutput) {
    if (newOutput < 0.1 && newOutput > -0.1) {
      return 0.0;
    } else {
      if (newOutput > AlgaeIntakeConstants.PIVOTMAXSPEED) {
        return AlgaeIntakeConstants.PIVOTMAXSPEED;
      } else if (newOutput < -AlgaeIntakeConstants.PIVOTMAXSPEED) {
        return -AlgaeIntakeConstants.PIVOTMAXSPEED;
      }
      else{
        return newOutput;
      }
    }
  }

  //runs the algae intake motor to a set speed
  public void runIntakeMotor(double speed) {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  //stops the algae intake motor
  public void stopIntakeMotor() {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, 0);
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

  //sets the new output of the algae pivot motor (manual control)
  public void setOutput(double newOutput){
    output = newOutput;
  }

  @Override
  public void periodic() {
    //prints the raw integrated encoder value of the pivot motor and if the PID is finished or not
    SmartDashboard.putNumber("[A] Pivot Encoder:", getEncoder());
    SmartDashboard.putBoolean("[A] isFinished?", isDone());

    //if PID is on, calculate the output of the pivot motor
    if (PIDOn) {
      output = pivotPID.calculate(getEncoder(), setpoint);

      //checks the output if it is within the max speeds
      if (output > AlgaeIntakeConstants.PIVOTMAXSPEED) {
        output = AlgaeIntakeConstants.PIVOTMAXSPEED;
      } else if (output < -AlgaeIntakeConstants.PIVOTMAXSPEED) {
        output = -AlgaeIntakeConstants.PIVOTMAXSPEED;
      }
      
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
    algaePivot.set(TalonSRXControlMode.PercentOutput, deadzone(output));

    //prints the output of the pivot motor, if the optical sensor is within sensing distance, and if the limit switch is pressed or not
    SmartDashboard.putNumber("[A] Pivot PID Output:", output);
    SmartDashboard.putBoolean("[A] Optical Sensor:", getOpticalValue());
    SmartDashboard.putBoolean("[A] Limit Switch:", getLSValue());

  }
}
