// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  private TalonFX elevatorMotor;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(0);
    topLimitSwitch = new DigitalInput(0);
    bottomLimitSwitch = new DigitalInput(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
