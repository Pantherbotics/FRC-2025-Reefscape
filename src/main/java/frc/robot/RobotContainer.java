// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.MoveEndEffector;
import frc.robot.commands.AlignToReef.ReefSide;
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
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.Visualizer;

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
  private final Vision vision = new Vision();
  public final Visualizer visualizer = new Visualizer(pivot, elevator, algaePivot, climber);





  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  .withDeadband(MetersPerSecond.of(0.18))
  .withRotationalDeadband(DegreesPerSecond.of(5))
  .withDriveRequestType(DriveRequestType.Velocity)
  .withSteerRequestType(SteerRequestType.MotionMagicExpo);

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
    elevator.setDefaultCommand(new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Stow")));
    // elevator.setDefaultCommand(Commands.idle(elevator));
    rollers.setDefaultCommand(rollers.setRollerSpeed(Volts.zero()));
    coralIntake.setDefaultCommand(coralIntake.setRollersVoltage(Volts.zero()).raceWith(Commands.waitSeconds(0.1)).andThen(coralIntake.disableServo()));
    algaeRoller.setDefaultCommand(algaeRoller.setVoltage(Volts.of(0)));
    algaePivot.setDefaultCommand(algaePivot.setAngleCommand(AlgaePivotConstants.kUpAngle).repeatedly());

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  
 

  
  private void configureBindings() {
    
    // joystick.leftBumper().onTrue(Commands.runOnce(()->SignalLogger.start()));
    // joystick.rightBumper().onTrue(Commands.runOnce(()->SignalLogger.stop()));

    // // algae pivot
    // joystick.start().and(joystick.a()).whileTrue(algaePivot.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(algaePivot.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(algaePivot.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(algaePivot.sysIdQuasistaticCommand(Direction.kReverse));
    
    // elevator
    // joystick.start().and(joystick.a()).whileTrue(elevator.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(elevator.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(elevator.sysIdQuasistaticCommand(Direction.kReverse));

    // // drivetrain
    // joystick.start().and(joystick.a()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // // pivot
    // joystick.start().and(joystick.a()).whileTrue(pivot.sysIdDynamicCommand(Direction.kForward));
    // joystick.start().and(joystick.b()).whileTrue(pivot.sysIdDynamicCommand(Direction.kReverse));
    // joystick.back().and(joystick.a()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kForward));
    // joystick.back().and(joystick.b()).whileTrue(pivot.sysIdQuasistaticCommand(Direction.kReverse));
    
    // Intake command
    joystick.leftBumper().and(()->!rollers.isSeated()).toggleOnTrue(
      Commands.race(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station")).andThen(
          coralIntake.setRollersVoltage(Volts.of(3.2))),
        rollers.seatCoral()
      ).withName("Coral station intake")
    );

    
    // L3 commands
    joystick.leftBumper().and(rollers::isSeated).onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L3"))
        .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getLeftBumperButton()),
        Commands.waitUntil(()->joystick.getHID().getLeftBumperButton()),
        Commands.waitUntil(()->!joystick.getHID().getLeftBumperButton()),
        rollers.setSpeedForSeconds(RollerConstants.kOuttakeVoltage, RollerConstants.kOuttakeTime),
        rollers.stopRollers()
      ).withName("L3 left bumper")
    );
    joystick.rightBumper().and(rollers::isSeated).onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L3"))
          .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getRightBumperButton()),
        Commands.waitUntil(()->joystick.getHID().getRightBumperButton()),
        Commands.waitUntil(()->!joystick.getHID().getRightBumperButton()),
        rollers.setSpeedForSeconds(RollerConstants.kOuttakeVoltage, RollerConstants.kOuttakeTime),
        rollers.stopRollers()
      ).withName("L3 right bumper")
    );

    // L2 commands
    joystick.start().and(rollers::isSeated).onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L2"))
          .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getStartButton()),
        Commands.waitUntil(()->joystick.getHID().getStartButton()),
        Commands.waitUntil(()->!joystick.getHID().getStartButton()),
        rollers.setSpeedForSeconds(RollerConstants.kOuttakeVoltage, RollerConstants.kOuttakeTime),
        rollers.stopRollers()
      ).withName("L2")
    );

    joystick.back().and(rollers::isSeated).onTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L2"))
          .raceWith(rollers.setRollerPosition(pivot::pivotAngle)),
        Commands.waitUntil(()->!joystick.getHID().getBackButton()),
        Commands.waitUntil(()->joystick.getHID().getBackButton()),
        Commands.waitUntil(()->!joystick.getHID().getBackButton()),
        rollers.setSpeedForSeconds(RollerConstants.kOuttakeVoltage, RollerConstants.kOuttakeTime),
        rollers.stopRollers()
      ).withName("L2")
    );

    // Alignment commands
    joystick.rightBumper().or(joystick.start()).and(rollers::isSeated).debounce(0.13, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.LEFT, false).withName("Left align"));
    joystick.leftBumper().or(joystick.back()).and(rollers::isSeated).debounce(0.13, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.RIGHT, false).withName("Right Align"));

    // Algae commands
    joystick.rightBumper().debounce(0.1, DebounceType.kRising).and(()->!rollers.hasCoral()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 2")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      ).withName("Algae removal 2")
    );

    joystick.start().debounce(0.1, DebounceType.kRising).and(()->!rollers.hasCoral()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 1")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      ).withName("Algae removal 1")
    );

    joystick.rightBumper().or(joystick.start()).and(()->!rollers.isSeated()).debounce(0.13, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.CENTER, false));


    // joystick.a().onTrue(
    //   Commands.sequence(
    //     new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L1"))
    //     .raceWith(rollers.setRollerPosition(pivot::pivotAngle))
    //   )
    // );

    joystick.leftTrigger().toggleOnTrue(
      Commands.sequence(
        algaePivot.setAngleCommand(AlgaePivotConstants.kDownAngle)
        .alongWith(algaeRoller.setVoltage(AlgaeRollerConstants.kIntakeVoltage)
        )
        //   .raceWith(algaeRoller.setVoltage(AlgaeRollerConstants.kIntakeVoltage)),
        // Commands.waitUntil(joystick.leftTrigger().negate()),
        // Commands.waitUntil(joystick.leftTrigger()),
        // algaeRoller.setVoltage(Volts.zero())
      ).withName("Algae collect")
    );

    joystick.rightTrigger().toggleOnTrue(
      algaePivot.setAngleCommand(AlgaePivotConstants.kDownAngle)
      .alongWith(algaeRoller.setVoltage(AlgaeRollerConstants.kOuttakeVoltage))
      .withName("Algae score")
    );

    joystick.povUp().onTrue(Commands.runOnce(()->drivetrain.resetRotation(Rotation2d.kZero)));

    joystick.x().debounce(0.25).onTrue(climber.setWinchPosition(ClimberConstants.kUpAngle).alongWith(coralIntake.setPulseWidth(1050)).raceWith(algaePivot.setAngleCommand(Degrees.of(80)).repeatedly()));
    joystick.b().onTrue(climber.setWinchPosition(Degrees.zero()).raceWith(algaePivot.setAngleCommand(Degrees.of(80)).repeatedly()));
// pressing anything that has b() as a joytstick thing also fires this ^
    NamedCommands.registerCommand("align left", new AlignToReef(drivetrain, ReefSide.LEFT, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("align center", new AlignToReef(drivetrain, ReefSide.CENTER, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("align right", new AlignToReef(drivetrain, ReefSide.RIGHT, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("position L2", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L2")).raceWith(rollers.setRollerPosition(pivot::pivotAngle)).withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("position L3", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L3")).raceWith(rollers.setRollerPosition(pivot::pivotAngle)).withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("shoot coral", rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage).raceWith(Commands.waitSeconds(0.5)));
    NamedCommands.registerCommand("remove algae 1", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 1")).andThen(rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("remove algae 2", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 2")).andThen(rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("zero elevator", elevator.zeroEncoder());
    NamedCommands.registerCommand("intake", 
    Commands.race(
      new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station")).andThen(
        coralIntake.setRollersVoltage(Volts.of(3.2))),
      rollers.seatCoral()
    ).withTimeout(5));
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


  public void updateVision(){

    var visionEsts = vision.getEstimatedGlobalPoses();
    for (Optional<EstimatedRobotPose> item: visionEsts){
      item.ifPresent(
        est -> {
            // Change our trust in the measurement based on the tags we can see
            vision.updateEstimationStdDevs(item, item.get().targetsUsed);
            var estStdDevs = vision.getEstimationStdDevs();

            drivetrain.addVisionMeasurement(
                    est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), estStdDevs);
        });
    }
        

  }

  public void updateSimVision(){
    vision.simulationPeriodic(drivetrain.getState().Pose);
    // var debugField = vision.getSimDebugField();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public Command moveAuto(){
    return drivetrain.applyRequest(()->new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(1, 0, 0))).deadlineFor(Commands.waitSeconds(3));
  }
}
