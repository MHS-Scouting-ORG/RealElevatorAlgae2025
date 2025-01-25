// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntake. */
  private TalonSRX algaeIntake;
  private TalonSRX algaePivot;
  private DigitalInput opticalSensor;
  private DigitalInput limitSwitch;

  public AlgaeIntakeSubsystem() {
    algaeIntake = new TalonSRX(0);
    algaePivot = new TalonSRX(0);
    opticalSensor = new DigitalInput(0);
    limitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
