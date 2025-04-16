// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecond;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class AlignToReef extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private ReefSide reefSide;
  private boolean endWhenClose;
  private Pose2d goalPose;
  private final int kRedIDoffset = 5;
  private final int kBlueIDoffset = 16;

  private ProfiledPIDController xController = DrivetrainConstants.kXController;
  private ProfiledPIDController yController = DrivetrainConstants.kYController;
  private ProfiledPIDController headingController = DrivetrainConstants.kHeadingController;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> topic = inst.getStructTopic("GoalPose", Pose2d.struct);
 private StructPublisher<Pose2d> pub = topic.publish();
  private Transform2d transform = Transform2d.kZero;

  /**
   * Auto alignment for reef scoring
   * @param drivetrain the drivetrain subsystem
   * @param isLeftSide True if the robot should align to the left reef branch
   * @param ReefSide which side of the reef the robot should align to, from 1-6. ID's start with the bottom right side and increase counterclockwise.
   * @param isRedAlliance True if the pose should be mirrored for the Blue alliance.
   */
  public enum ReefSide{
    LEFT,
    RIGHT,
    CENTER
  }
  public AlignToReef(CommandSwerveDrivetrain drivetrain, ReefSide side, boolean endWhenClose) {
    this.drivetrain = drivetrain;
    this.reefSide = side;
    this.endWhenClose = endWhenClose;
    addRequirements(drivetrain);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    switch (reefSide){
      case LEFT:
        this.transform = VisionConstants.kLeftTransform;
        break;
      case RIGHT:
        this.transform = VisionConstants.kRightTransform;
        break;
      case CENTER:
        this.transform = VisionConstants.kCenterTransform;
        break;
    }

    Pose2d robotPose = drivetrain.getState().Pose;
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, robotPose.getRotation());
    goalPose = getClosestTagPose(robotPose).plus(transform);
    pub.set(goalPose);
    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    headingController.setGoal(goalPose.getRotation().getRadians());
    xController.reset(new State(robotPose.getX(), fieldSpeeds.vxMetersPerSecond));
    yController.reset(new State(robotPose.getY(), fieldSpeeds.vyMetersPerSecond));
    headingController.reset(new State(robotPose.getRotation().getRadians(), fieldSpeeds.omegaRadiansPerSecond));
  }

  private Pose2d getClosestTagPose(Pose2d robotPose){
    var tags =VisionConstants.kAprilTagLayout.getTags();
    List<Pose2d> tagPoses = tags.stream()
                .filter(tag -> (tag.ID > kRedIDoffset && tag.ID <= kRedIDoffset+6) || (tag.ID > kBlueIDoffset && tag.ID <= kBlueIDoffset+6))
                .map(tag -> tag.pose.toPose2d())
                .collect(Collectors.toList());

    return robotPose.nearest(tagPoses);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drivetrain.getState().Pose;

    drivetrain.setControl(new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withVelocityX(xController.calculate(robotPose.getX()))
      .withVelocityY(yController.calculate(robotPose.getY()))
      .withRotationalRate(headingController.calculate(robotPose.getRotation().getRadians()))
      .withDeadband(InchesPerSecond.of(10))
      .withRotationalDeadband(DegreesPerSecond.of(15))
      .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    );
  }

  @Override
  public void end(boolean interrupted) {
    if (this.endWhenClose){
      drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endWhenClose && drivetrain.getState().Pose.relativeTo(goalPose).getTranslation().getNorm() < VisionConstants.kDistToleranceMeters;
  }
}
