package frc.robot.subsystems;

import java.util.function.ToLongBiFunction;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private TalonSRX algaeIntake, algaePivot;
  private DigitalInput opticalSensor;

  private PIDController pivotPID;
  private boolean PIDOn;
  private double previousError;
  private double currentError;
  private double output;
  private double setpoint;

  public AlgaeIntakeSubsystem() {
    algaeIntake = new TalonSRX(AlgaeIntakeConstants.INTAKEID);
    algaeIntake.configFactoryDefault();

    opticalSensor = new DigitalInput(AlgaeIntakeConstants.OPTICALID);

    algaeIntake.setNeutralMode(NeutralMode.Brake);
    algaeIntake.enableCurrentLimit(AlgaeIntakeConstants.CURRENTLIMIT);
    algaeIntake.configPeakCurrentLimit(25);
    algaeIntake.configPeakCurrentDuration(100, 50);
    algaeIntake.configContinuousCurrentLimit(20);

    algaePivot = new TalonSRX(AlgaeIntakeConstants.PIVOTID);

    pivotPID = new PIDController(AlgaeIntakeConstants.KP, AlgaeIntakeConstants.KI, AlgaeIntakeConstants.KD);
    pivotPID.setTolerance(AlgaeIntakeConstants.TOLERANCE);

    algaePivot.setNeutralMode(NeutralMode.Brake);
    algaePivot.enableCurrentLimit(AlgaeIntakeConstants.CURRENTLIMIT);
    algaePivot.configPeakCurrentLimit(25);
    algaePivot.configPeakCurrentDuration(100, 50);
    algaePivot.configContinuousCurrentLimit(20);

    PIDOn = false;
    output = 0.0;
  }

  // returns output of motor
  public double getOutput() {
    return output;
  }

  // returns the value of the optical sensor (true or false)
  public boolean getOpticalValue() {
    return opticalSensor.get();
  }

  public double getIntakeAmps() {
    return algaeIntake.getBusVoltage();
  }

  // runs the algae intake motor to a set speed
  public void runIntakeMotor(double speed) {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  // stops the algae intake motor
  public void stopIntakeMotor() {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, 0);
  }

  // returns the raw integrated encoder value of the algae pivot motor
  public double getEncoder() {
    return algaePivot.getSensorCollection().getQuadraturePosition();
  }

  // resets the algae pivot integrated encoder to 0
  void resetEncoder() {
    algaePivot.getSensorCollection().setQuadraturePosition(0, 0);
  }

  // returns the value of the limit switch (true or false)
  public boolean getLSValue() {
    if (algaeIntake.isFwdLimitSwitchClosed() == 1) {
      return true;
    } else {
      return false;
    }
  }

  // stops the algae pivot motor
  public void stopPivotMotor() {
    algaePivot.set(TalonSRXControlMode.PercentOutput, 0);
  }

  // turns the PID on
  public void enablePID() {
    PIDOn = true;
  }

  // turns the PID off
  public void disablePID() {
    PIDOn = false;
  }

  // resets the I value on overshoot
  private void resetI() {
    currentError = pivotPID.getPositionError();

    if (currentError > 0 && previousError < 0) {
      pivotPID.reset();
    } 
    else if (currentError < 0 && previousError > 0) {
      pivotPID.reset();
    }

    previousError = currentError;
  }

  // returns if the PID is finished (PID is at setpoint or not)
  public boolean isDone() {
    return pivotPID.atSetpoint();//(AlgaeIntakeConstants.TOLERANCE > Math.abs(currentError));
  }

  // sets the new setpoint for the PID
  public void setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
  }

  // sets the new output of the algae pivot motor (manual control) and checks the
  // output to see if it's within the max speed parameters
  public void setOutput(double newOutput) {
    output = newOutput;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("[A] Optical Sensor:", getOpticalValue());
    SmartDashboard.putNumber("[A] AMPs", getIntakeAmps());
    SmartDashboard.putNumber("[A] test", algaeIntake.isFwdLimitSwitchClosed());

    // prints the raw integrated encoder value of the pivot motor, if the PID is
    // finished or not
    SmartDashboard.putNumber("[A] Pivot Encoder:", getEncoder());
    SmartDashboard.putBoolean("[A] PidOn?", PIDOn);
    SmartDashboard.putBoolean("[A] isFinished?", isDone());
    SmartDashboard.putData("[A] PID Controller", pivotPID);

    //reset the algae pivot encoders if touching the limit switch
    if(getLSValue()){
      resetEncoder();
    }

    resetI();

    // if PID is on, calculate the output of the pivot motor
    if (PIDOn) { 
      output = pivotPID.calculate(getEncoder(), setpoint);


    }

    if (output > AlgaeIntakeConstants.PIVOTMAXSPEED) {
      output = AlgaeIntakeConstants.PIVOTMAXSPEED;
    } else if (output < -AlgaeIntakeConstants.PIVOTMAXSPEED) {
      output = -AlgaeIntakeConstants.PIVOTMAXSPEED;
    }

    // checks the output of the pivot motor and sets it to the motor if it is within
    // the deadzone and max speeds
    algaePivot.set(TalonSRXControlMode.PercentOutput, output);

    // prints the output of the pivot motor and if the limit switch is pressed or
    // not
    SmartDashboard.putNumber("[A] Pivot PID Output:", output);
    SmartDashboard.putBoolean("[A] Limit Switch:", getLSValue());

  }
}
