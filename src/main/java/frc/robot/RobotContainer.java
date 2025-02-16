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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotStates;
import frc.robot.commands.MoveEndEffector;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Rollers.Rollers;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final Pivot pivot = new Pivot();
  private final Rollers rollers = new Rollers();
  private final Telemetry telemetry = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  public RobotContainer() {
    drivetrain.registerTelemetry(telemetry::telemeterize);
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(()->
      drive.withVelocityX(joystick.getLeftY()).withVelocityY(joystick.getLeftX()).withRotationalRate(-joystick.getRightX()*2)
      )
    );
    //elevator.setDefaultCommand(new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Stow")));

    configureBindings();
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp){
    drivetrain.addVisionMeasurement(pose, timestamp);
  }


  private void configureBindings() {
    
    joystick.leftBumper().onTrue(Commands.runOnce(()->SignalLogger.start()));
    joystick.rightBumper().onTrue(Commands.runOnce(()->SignalLogger.stop()));

    // Elevator
    // joystick.start().and(joystick.a()).whileTrue(elevator.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(elevator.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kReverse));

    // drivetrain
    // joystick.start().and(joystick.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
    // pivot
    joystick.start().and(joystick.a()).whileTrue(pivot.sysIdDynamicCommand(Direction.kForward));
    joystick.start().and(joystick.b()).whileTrue(pivot.sysIdDynamicCommand(Direction.kReverse));
    joystick.back().and(joystick.a()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kForward));
    joystick.back().and(joystick.b()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kReverse));

    
    //joystick.povUp().onTrue(new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L1")));
    // joystick.x().onTrue(elevator.setHeightCommand(Inches.of(18)));
    // joystick.y().onTrue(elevator.setHeightCommand(Inches.of(2)));
    // joystick.a().onTrue(elevator.setHeightCommand(Inches.of(15)));
    // joystick.b().onTrue(elevator.setHeightCommand(Inches.of(29)));

    // joystick.x().onTrue(pivot.setAngleCommand(Degrees.of(90)));
    // joystick.y().onTrue(pivot.setAngleCommand(Degrees.of(0)));

    joystick.povDown().onTrue(elevator.zeroEncoder());



  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
