// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCmds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCmd extends Command {
  AlgaeIntakeSubsystem algaeIntakeSub;
  Timer timer;
  boolean seen;

  public IntakeCmd(AlgaeIntakeSubsystem newAlgaeIntakeSubs) {
    algaeIntakeSub = newAlgaeIntakeSubs;
    timer = new Timer();
    seen = false;
    addRequirements(algaeIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("[A] Start", "yes");
    timer.reset();
    seen = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntakeSub.runIntakeMotor(AlgaeIntakeConstants.INTAKEMAXSPEED);

    if (algaeIntakeSub.getOpticalValue() && !seen){
      timer.start();
      seen = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("[A] END", "yes");
    algaeIntakeSub.runIntakeMotor(0.4);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= 1;
  }
}
