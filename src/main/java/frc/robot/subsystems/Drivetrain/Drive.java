// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
 
  private final CommandSwerveDrivetrain drivetrain;
  private final XboxController controller;
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DrivetrainConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DrivetrainConstants.kMaxAcceleration.in(MetersPerSecondPerSecond));
 
  public Drive(CommandSwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain=drivetrain;
    this.controller=controller;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    var xVelocity = -controller.getLeftY() * DrivetrainConstants.kMaxSpeed.in(MetersPerSecond);
    var yVelocity = -controller.getLeftX() * DrivetrainConstants.kMaxSpeed.in(MetersPerSecond);
    drivetrain.setControl(
      DrivetrainConstants.drive
      .withVelocityX(xLimiter.calculate(xVelocity))
      .withVelocityY(yLimiter.calculate(yVelocity))
      .withRotationalRate(-controller.getRightX() * DrivetrainConstants.MaxAngularRate.in(RadiansPerSecond))
    );
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
