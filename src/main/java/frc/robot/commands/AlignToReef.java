// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class AlignToReef extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final boolean isLeft;

  public AlignToReef(CommandSwerveDrivetrain drivetrain, boolean isLeft) {
    this.drivetrain = drivetrain;
    this.isLeft = isLeft;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
