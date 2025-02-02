// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.AlgaeIntakeCmds.ManualPivotCmd;
import frc.robot.commands.ElevatorCmds.ManualElevatorCmd;
import frc.robot.commands.ElevatorCmds.TestPIDCmd;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.commands.AlgaeIntakeCmds.ManualPivotCmd;
import frc.robot.commands.AlgaeIntakeCmds.StoragePositionCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);

  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  public RobotContainer() {
 
    elevatorSubsystem.setDefaultCommand(new ManualElevatorCmd(elevatorSubsystem, () -> xbox.getLeftY()));

    //joystick control for moving the algae pivot manually
    algaeIntakeSubsystem.setDefaultCommand(new ManualPivotCmd(algaeIntakeSubsystem, () -> xbox.getRightY()));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new TestPIDCmd(elevatorSubsystem, 100));
    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new TestPIDCmd(elevatorSubsystem, 0));
    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new StoragePositionCmd(algaeIntakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto();
  }
}
