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

  private double speed;

  private boolean PIDOn = false;
  private double output = 0.0;
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

  //returns the integrated encoder value of the algae pivot motor 
  public double getEncoder() {
    return algaePivot.getSelectedSensorPosition() / 1000;
  }

  //returns the value of the optical sensor (true or false)
  public boolean getOpticalValue() {
    return opticalSensor.get();
  }

  //returns the value of the limit switch (true or false)
  public boolean getLSValue() {
    return limitSwitch.get();
  }

  //checks if the speed of the pivot motor is within the deadzone and max speed parameters
  void deadzone() {
    if (speed < 0.1 && speed > -0.1) {
      speed = 0.0;
    } else {
      if (speed > AlgaeIntakeConstants.PIVOTMAXSPEED) {
        speed = AlgaeIntakeConstants.PIVOTMAXSPEED;
      } else if (speed < -AlgaeIntakeConstants.PIVOTMAXSPEED) {
        speed = -AlgaeIntakeConstants.PIVOTMAXSPEED;
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

  //sets the new speed of the algae pivot motor (manual control)
  public void setSpeed(double newSpeed){
    speed = newSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("[A] Pivot Encoder:", getEncoder());
    SmartDashboard.putBoolean("[A] isFinished?", isDone());

    if (PIDOn) {
      output = pivotPID.calculate(getEncoder(), setpoint);

      if (output > AlgaeIntakeConstants.PIVOTMAXSPEED) {
        output = AlgaeIntakeConstants.PIVOTMAXSPEED;
      } else if (output < -AlgaeIntakeConstants.PIVOTMAXSPEED) {
        output = -AlgaeIntakeConstants.PIVOTMAXSPEED;
      }

      if (isDone() || (getLSValue() && output < 0)) {
        disablePID();
        stopPivotMotor();
      } else {
        algaePivot.set(TalonSRXControlMode.Current, output);
      }
    } else {
      if (getLSValue() && speed < 0) {
        stopPivotMotor();
      } else {
        deadzone();
        algaePivot.set(TalonSRXControlMode.PercentOutput, speed);
      }
    }

    SmartDashboard.putNumber("[A] Pivot PID Output:", output);
    SmartDashboard.putBoolean("[A] Optical Sensor:", getOpticalValue());
    SmartDashboard.putBoolean("[A] Limit Switch:", getLSValue());

  }
}
