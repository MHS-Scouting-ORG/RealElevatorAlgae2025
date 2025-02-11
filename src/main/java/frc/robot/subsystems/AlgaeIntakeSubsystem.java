package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  private TalonSRX algaeIntake;
  private DigitalInput opticalSensor;

  public AlgaeIntakeSubsystem() {
    algaeIntake = new TalonSRX(AlgaeIntakeConstants.INTAKEID);

    opticalSensor = new DigitalInput(AlgaeIntakeConstants.OPTICALID);

    algaeIntake.setNeutralMode(NeutralMode.Brake);
    algaeIntake.enableCurrentLimit(AlgaeIntakeConstants.CURRENTLIMIT);
    algaeIntake.configPeakCurrentLimit(35);
    algaeIntake.configPeakCurrentDuration(100, 50);
    algaeIntake.configContinuousCurrentLimit(30);
    }

  //returns the value of the optical sensor (true or false)
  public boolean getOpticalValue() {
    return !opticalSensor.get();
  }

  public double getIntakeAmps(){
    return algaeIntake.getBusVoltage();
  }

  //runs the algae intake motor to a set speed
  public void runIntakeMotor(double speed) {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, speed);
  }

  //stops the algae intake motor
  public void stopIntakeMotor() {
    algaeIntake.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("[A] Optical Sensor:", getOpticalValue());
    SmartDashboard.putNumber("[A] AMPs", getIntakeAmps());

  }
}
