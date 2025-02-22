// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.pathplanner.lib.auto.AutoBuilder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.MoveEndEffector;
import frc.robot.subsystems.AlgaeIntake.AlgaePivot;
import frc.robot.subsystems.AlgaeIntake.AlgaeRoller;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.CoralIntake.CoralIntake;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.Pivot;
import frc.robot.subsystems.EndEffector.Rollers;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final Pivot pivot = new Pivot();
  private final Rollers rollers = new Rollers();
  private final CoralIntake coralIntake = new CoralIntake();
  private final Telemetry telemetry = new Telemetry(DrivetrainConstants.kMaxSpeed.in(MetersPerSecond));
  private final Climber climber = new Climber();
  private final AlgaePivot algaePivot = new AlgaePivot();
  private final AlgaeRoller algaeRoller = new AlgaeRoller();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MetersPerSecond.of(0.18
  ))
  .withRotationalDeadband(DegreesPerSecond.of(5))
  .withDriveRequestType(DriveRequestType.Velocity)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final SendableChooser <Command> autoChooser;

  public RobotContainer() {
    drivetrain.registerTelemetry(telemetry::telemeterize);
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(()->drive
        .withVelocityX(-joystick.getLeftY() * DrivetrainConstants.kMaxSpeed.in(MetersPerSecond))
        .withVelocityY(-joystick.getLeftX() * DrivetrainConstants.kMaxSpeed.in(MetersPerSecond))
        .withRotationalRate(-joystick.getRightX() * DrivetrainConstants.kMaxRotationRate.in(RadiansPerSecond))
      )
    );
    elevator.setDefaultCommand(new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Stow")).repeatedly());
    // elevator.setDefaultCommand(Commands.idle(elevator));
    rollers.setDefaultCommand(rollers.setRollerSpeed(Volts.zero()));
    coralIntake.setDefaultCommand(coralIntake.setRollersVoltage(Volts.zero()).raceWith(Commands.waitSeconds(0.1)).andThen(coralIntake.disableServo()));
    algaeRoller.setDefaultCommand(algaeRoller.setVoltage(Volts.of(0)));
    algaePivot.setDefaultCommand(algaePivot.setAngleCommand(AlgaePivotConstants.kUpAngle));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  public void addVisionMeasurement(Pose2d pose, double timestamp){
    drivetrain.addVisionMeasurement(pose, timestamp);
  }


  private void configureBindings() {
    
    // joystick.leftBumper().onTrue(Commands.runOnce(()->SignalLogger.start()));
    // joystick.rightBumper().onTrue(Commands.runOnce(()->SignalLogger.stop()));

    // joystick.start().and(joystick.a()).whileTrue(algaePivot.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(algaePivot.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(algaePivot.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(algaePivot.sysIdQuasistaticCommand(Direction.kReverse));
    

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
    // joystick.start().and(joystick.a()).whileTrue(pivot.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(pivot.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kReverse));

    // joystick.a().onTrue(algaePivot.setAngleCommand(Degrees.of(0)));
    // joystick.b().onTrue(algaePivot.setAngleCommand(Degrees.of(30)));
    // joystick.x().onTrue(algaePivot.setAngleCommand(Degrees.of(50)));
    // joystick.y().onTrue(algaePivot.setAngleCommand(Degrees.of(90)));

    joystick.leftBumper().or(joystick.rightBumper()).and(rollers::hasCoral).onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L3"))
          .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getRightBumperButton() & !joystick.getHID().getLeftBumperButton()),
        Commands.waitUntil(()->joystick.getHID().getRightBumperButton() || joystick.getHID().getLeftBumperButton()),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage).raceWith(Commands.waitSeconds(0.5)),
        rollers.setRollerSpeed(Volts.zero()).raceWith(Commands.waitSeconds(0.1))
      )
    );

    joystick.leftBumper().and(()->!rollers.hasCoral()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 1")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      )
    );
    joystick.rightBumper().and(()->!rollers.hasCoral()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 2")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      )
    );
    

    joystick.rightBumper().and(rollers::hasCoral).debounce(0.33, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, false, 1, false));
    joystick.leftBumper().and(rollers::hasCoral).debounce(0.33, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, true, 1, false));

    joystick.start().onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L2"))
          .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getStartButton()),
        Commands.waitUntil(()->joystick.getHID().getStartButton()),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage).raceWith(Commands.waitSeconds(0.5)),
        rollers.setRollerSpeed(Volts.zero()).raceWith(Commands.waitSeconds(0.1))
      )
    );

    joystick.back().toggleOnTrue(
      Commands.race(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station")).andThen(Commands.idle()),
        rollers.setRollerSpeed(RollerConstants.kIntakeVoltage).until(rollers::hasCoral)
          .andThen(rollers.setRollerSpeed(RollerConstants.kSeatVoltage).until(()->!rollers.hasCoral()))
          .andThen(rollers.setRollerSpeed(RollerConstants.kBackVoltage).until(rollers::hasCoral))
          .andThen(rollers.setRollerSpeed(Volts.zero()).raceWith(Commands.waitSeconds(0.1))),
        coralIntake.setRollersVoltage(Volts.of(3.2)).andThen(coralIntake.setPulseWidth(1700)).repeatedly()
      )
    );

    joystick.a().onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L1")),
        rollers.setRollerPosition(pivot::pivotAngle)
      )
    );

    joystick.leftTrigger().toggleOnTrue(
      algaePivot.setAngleCommand(AlgaePivotConstants.kDownAngle)
      .alongWith(algaeRoller.setVoltage(AlgaeRollerConstants.kIntakeVoltage))
    );

    joystick.povUp().onTrue(Commands.runOnce(()->drivetrain.resetRotation(Rotation2d.kZero)));

    joystick.povLeft().onTrue(climber.setWinchPosition(ClimberConstants.kUpAngle).alongWith(coralIntake.setPulseWidth(1050)).raceWith(algaePivot.setAngleCommand(Degrees.of(80)).repeatedly()));
    joystick.povRight().onTrue(climber.setWinchPosition(Degrees.zero()).raceWith(algaePivot.setAngleCommand(Degrees.of(80)).repeatedly()));

    // joystick.y().onTrue(
    //   Commands.parallel(
    //     climber.setWinchPosition(ClimberConstants.kUpAngle),
    //     climber.setServoLocked(true)
    //   )
    // );
    // the thing above does not work as it requires the same subsystem, how to fix? -eric

    // joystick.a().onTrue(climber.setWinchPosition(Degrees.zero()));

    joystick.povDown().onTrue(elevator.zeroEncoder());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}