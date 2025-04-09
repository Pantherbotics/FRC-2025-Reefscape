// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.RobotStates;
import frc.robot.Constants.RollerConstants;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.AlignToReef.ReefSide;
import frc.robot.commands.MoveEndEffector;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drivetrain.Telemetry;
import frc.robot.subsystems.Drivetrain.TunerConstants;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.Pivot;
import frc.robot.subsystems.EndEffector.Rollers;
import frc.robot.subsystems.GroundIntake.GroundPivot;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.Visualizer;

public class RobotContainer {
  private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Elevator elevator = new Elevator();
  private final Pivot pivot = new Pivot();
  private final Rollers rollers = new Rollers();
  private final Telemetry telemetry = new Telemetry(DrivetrainConstants.kMaxSpeed.in(MetersPerSecond));
  private final Climber climber = new Climber();
  private final GroundPivot groundPivot = new GroundPivot();
  private final Vision vision = new Vision();
  public final Visualizer visualizer = new Visualizer(pivot, elevator, groundPivot, climber);





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
    rollers.setDefaultCommand(rollers.setRollerPosition(pivot::pivotAngle));
    groundPivot.setDefaultCommand(groundPivot.setAngleCommand(AlgaePivotConstants.kUpAngle).repeatedly());

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
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station")),
          rollers.setRollerSpeed(RollerConstants.kIntakeVoltage).until(rollers::hasCoral).andThen(rollers.smartIntake())
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
    joystick.rightBumper().or(joystick.start()).and(rollers::isSeated).debounce(0.13, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.RIGHT, false).withName("Left align"));
    joystick.leftBumper().or(joystick.back()).and(rollers::isSeated).debounce(0.13, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.LEFT, false).withName("Right Align"));

    // Algae commands
    joystick.rightBumper().and(()->!rollers.isSeated() && !rollers.isSeating()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 2")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      ).withName("Algae removal 2")
    );

    joystick.start().and(()->!rollers.isSeated() && !rollers.isSeating()).toggleOnTrue(
      Commands.sequence(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 1")),
        rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage)
      ).withName("Algae removal 1")
    );

    joystick.back().and(()->!rollers.isSeated()).onTrue(
      rollers.seatCoral()
    );

    joystick.rightBumper().or(joystick.start()).and(()->!rollers.isSeated()).debounce(0.2, DebounceType.kRising).whileTrue(new AlignToReef(drivetrain, ReefSide.CENTER, false));
    // joystick.a().onTrue(
    //   Commands.sequence(
    //     new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L1"))
    //     .raceWith(rollers.setRollerPosition(pivot::pivotAngle))
    //   
    // );


    joystick.povLeft().toggleOnTrue(
      rollers.setRollerPosition(()->Rotations.of((joystick.getRightTriggerAxis()-0.5)*0.5))
    );
      
    joystick.povUp().onTrue(Commands.runOnce(()->drivetrain.resetRotation(Rotation2d.kZero)));
      
    joystick.a().onTrue(drivetrain.wheelRadiusCharacterization());
    joystick.leftStick().and(joystick.rightStick()).debounce(1,DebounceType.kRising).onTrue(
      drivetrain.superMode()
    ).onTrue(
      Commands.run(()->joystick.getHID().setRumble(RumbleType.kBothRumble, Math.hypot(drivetrain.getPigeon2().getAccelerationX().getValueAsDouble(), drivetrain.getPigeon2().getAccelerationY().getValueAsDouble())/2.83))
      // Commands.run(()->joystick.getHID().setRumble(RumbleType.kBothRumble, Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond)/5))
    );

    joystick.x().debounce(0.25).onTrue(climber.setWinchPosition(ClimberConstants.kUpAngle).raceWith(groundPivot.setAngleCommand(Degrees.of(80)).repeatedly()));
    joystick.b().onTrue(climber.setWinchPosition(Rotations.of(20)).raceWith(groundPivot.setAngleCommand(Degrees.of(80)).repeatedly()));
    joystick.y().whileTrue(climber.setVoltage(Volts.of(12))).onFalse(climber.setVoltage(Volts.zero()));
// pressing anything that has b() as a joytstick thing also fires this ^
    NamedCommands.registerCommand("align left", new AlignToReef(drivetrain, ReefSide.LEFT, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("align center", new AlignToReef(drivetrain, ReefSide.CENTER, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("align right", new AlignToReef(drivetrain, ReefSide.RIGHT, true).withTimeout(Seconds.of(3)));
    NamedCommands.registerCommand("position L2", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L2")).raceWith(rollers.setRollerPosition(pivot::pivotAngle)).withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("position L3", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("L3")).raceWith(rollers.setRollerPosition(pivot::pivotAngle)).withTimeout(Seconds.of(2)));
    NamedCommands.registerCommand("shoot coral", rollers.setRollerSpeed(RollerConstants.kOuttakeVoltage).raceWith(Commands.waitSeconds(0.3)));
    NamedCommands.registerCommand("remove algae 1", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 1")).andThen(rollers.setRollerSpeed(RollerConstants.kAlgaeRemovalVoltage)).withTimeout(Seconds.of(2.5)));
    NamedCommands.registerCommand("remove algae 2", new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("Algae 2")).andThen(rollers.setRollerSpeed(RollerConstants.kAlgaeRemovalVoltage)).withTimeout(Seconds.of(2.5)));
    NamedCommands.registerCommand("zero elevator", elevator.zeroEncoder());
    NamedCommands.registerCommand("intake", 
      Commands.race(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station")),
        rollers.setRollerSpeed(RollerConstants.kIntakeVoltage).until(rollers::hasCoral).andThen(rollers.seatCoral())
      ).withTimeout(4));
    NamedCommands.registerCommand("intake with current check",
      Commands.parallel(
        new MoveEndEffector(elevator, pivot, RobotStates.EEStates.get("coral station"))
      )
    );
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
