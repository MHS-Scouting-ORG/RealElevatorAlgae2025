// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaePivotCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

//Algae Pivot: Processor Position and Pick Up Off the Ground Position are the same position
public class ProcessorPositionCmd extends Command {
  AlgaeIntakeSubsystem algaeIntakeSub;

  public ProcessorPositionCmd(AlgaeIntakeSubsystem newAlgaeIntakeSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    addRequirements(algaeIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.reset();
    algaeIntakeSub.setSetpoint(-1100); //-1350
    algaeIntakeSub.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
