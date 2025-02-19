// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaePivotCmds;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NoAlgaeTuckCmd extends Command {
  AlgaeIntakeSubsystem algaeIntakeSub;
  public NoAlgaeTuckCmd(AlgaeIntakeSubsystem newAlgaeIntakeSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    algaeIntakeSub = newAlgaeIntakeSub;
    addRequirements(algaeIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntakeSub.setSetpoint(20);
    // algaeIntakeSub.disablePID();
    // algaeIntakeSub.setOutput(0.3);
    // algaeIntakeSub.setSetpoint(5);
    // algaeIntakeSub.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntakeSub.setOutput(0.3);
    // if(algaeIntakeSub.getLSValue()){
    //   algaeIntakeSub.disablePID();
    //   algaeIntakeSub.setOutput(0.1);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSub.disablePID();
    algaeIntakeSub.setOutput(0.15);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeIntakeSub.getLSValue() || algaeIntakeSub.isDone();
  }
}
