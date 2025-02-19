// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaePivotCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DealgifyL3PositionCmd extends Command {
  AlgaeIntakeSubsystem algaeIntakeSub;
  public DealgifyL3PositionCmd(AlgaeIntakeSubsystem newAlgaeIntakeSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    addRequirements(algaeIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntakeSub.disablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntakeSub.setOutput(-1.0);
    algaeIntakeSub.runIntakeMotor(-AlgaeIntakeConstants.INTAKEMAXSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSub.setOutput(0.0);
    algaeIntakeSub.stopIntakeMotor();
    algaeIntakeSub.setSetpoint(40);
    algaeIntakeSub.enablePID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeIntakeSub.getEncoder() < -1600;
  }
}
