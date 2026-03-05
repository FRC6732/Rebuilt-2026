// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.FireSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PlowSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  final CommandXboxController m_driverController = new CommandXboxController(0);
  private final SwerveSubsystem m_drivebase = new SwerveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final FireSubsystem m_cannon = new FireSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final PlowSubsystem m_plow = new PlowSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivebase.setDefaultCommand(m_drivebase.driveCommand(
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * -1,
      () -> m_driverController.getRightX() * -1,
      true
    ));

    m_driverController.a().toggleOnTrue(m_intake.Intake());
    m_driverController.x().whileTrue(m_hopper.Hopper());
    m_driverController.y().toggleOnTrue(m_cannon.Fire());
    m_driverController.rightTrigger().whileTrue(m_plow.PlowUp());
    m_driverController.leftTrigger().whileTrue(m_plow.PlowDown());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
