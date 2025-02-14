// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Elevator.Elevator;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final Telemetry telemetry = new Telemetry(6);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  public RobotContainer() {
    drivetrain.registerTelemetry(telemetry::telemeterize);
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(()->
      drive.withVelocityX(joystick.getLeftY()).withVelocityY(joystick.getLeftX()).withRotationalRate(-joystick.getRightX()*2)
      )
    );

    configureBindings();
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp){
    drivetrain.addVisionMeasurement(pose, timestamp);
  }


  private void configureBindings() {
    joystick.leftBumper().onTrue(Commands.runOnce(()->SignalLogger.start()));
    joystick.rightBumper().onTrue(Commands.runOnce(()->SignalLogger.stop()));
    joystick.start().and(joystick.a()).whileTrue(elevator.sysIdDynamicCommand(Direction.kForward));
    joystick.start().and(joystick.b()).whileTrue(elevator.sysIdDynamicCommand(Direction.kReverse));
    joystick.back().and(joystick.a()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kForward));
    joystick.back().and(joystick.b()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kReverse));

    joystick.x().onTrue(elevator.setHeightCommand(Inches.of(20)));
    joystick.y().onTrue(elevator.setHeightCommand(Inches.of(2)));

    joystick.povDown().onTrue(elevator.zeroEncoder());



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
