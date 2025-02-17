// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevProcessorPos extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private Timer timer;

  /** Creates a new TestPIDCmda. */
  public ElevProcessorPos(ElevatorSubsystem newElevatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    elevatorSubsystem = newElevatorSubsystem;
    timer = new Timer();

    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.turnPIDOn();
    elevatorSubsystem.setSetpoint(31);// 48 is pick off algae coral

    timer.stop();
    timer.reset();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevatorSubsystem.atSetpoint()){
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Endede", "It hs eneded");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 0.3;
  }
}
