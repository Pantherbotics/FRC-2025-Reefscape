// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class AlignToReef extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private boolean isLeftSide;
  private Pose2d goalPose;
  private boolean end = false;
  private final int kRedIDoffset = 5;
  private final int kBlueIDoffset = 16;

  // private PIDController xController = DrivetrainConstants.kXController;
  // private PIDController yController = DrivetrainConstants.kYController;
  // private PIDController headingController = DrivetrainConstants.kHeadingController;
  private PPHolonomicDriveController cont = new PPHolonomicDriveController(DrivetrainConstants.kTranslationConstants, DrivetrainConstants.kHeadingConstants);
  private PathPlannerTrajectoryState goal = new PathPlannerTrajectoryState();

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> topic = inst.getStructTopic("GoalPose", Pose2d.struct);
  private StructPublisher<Pose2d> pub = topic.publish();
  private final double kMaxTranslationSpeed = 4;
  private final double kTranslationDeadband = 0.05;
  private final double kMaxRotationSpeed = 1.5 * Math.PI;
  private final double kRotationDeadband = 0.2;

  /**
   * Auto alignment for reef scoring
   * @param drivetrain the drivetrain subsystem
   * @param isLeftSide True if the robot should align to the left reef branch
   * @param reefSide which side of the reef the robot should align to, from 1-6. ID's start with the bottom right side and increase counterclockwise.
   * @param isRedAlliance True if the pose should be mirrored for the Blue alliance.
   */
  public AlignToReef(CommandSwerveDrivetrain drivetrain, boolean isLeftSide) {
    this.drivetrain = drivetrain;
    this.isLeftSide = isLeftSide;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    goalPose = getClosestTagPose(drivetrain.getState().Pose).plus(isLeftSide?VisionConstants.kLeftTransform:VisionConstants.kRightTransform);
    pub.set(goalPose);
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
    goal.pose = goalPose;
    var speeds = cont.calculateRobotRelativeSpeeds(drivetrain.getState().Pose, goal);
    speeds.vxMetersPerSecond = MathUtil.clamp(MathUtil.applyDeadband(speeds.vxMetersPerSecond, kTranslationDeadband), -kMaxTranslationSpeed, kMaxTranslationSpeed);
    speeds.vyMetersPerSecond =  MathUtil.clamp(MathUtil.applyDeadband(speeds.vyMetersPerSecond, kTranslationDeadband), -kMaxTranslationSpeed, kMaxTranslationSpeed);
    speeds.omegaRadiansPerSecond = MathUtil.clamp(MathUtil.applyDeadband(speeds.omegaRadiansPerSecond, kRotationDeadband), -kMaxRotationSpeed, kMaxRotationSpeed);
    // ChassisSpeeds speeds = new ChassisSpeeds(
    //   xController.calculate(drivetrain.getState().Pose.getX()),
    //   yController.calculate(drivetrain.getState().Speeds.vyMetersPerSecond),
    //   headingController.calculate(drivetrain.getState().Speeds.omegaRadiansPerSecond)
    // );
    drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.Velocity)
      .withSpeeds(speeds)
    );
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
