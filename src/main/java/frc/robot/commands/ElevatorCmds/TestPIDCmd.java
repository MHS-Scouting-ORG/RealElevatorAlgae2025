// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestPIDCmd extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private double setpoint;

  /** Creates a new TestPIDCmd. */
  public TestPIDCmd(ElevatorSubsystem newElevatorSubsystem, double newSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.

    elevatorSubsystem = newElevatorSubsystem;
    setpoint = newSetpoint;
    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.turnPIDOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    elevatorSubsystem.setSetpoint(setpoint);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atSetpoint();
  }
}
