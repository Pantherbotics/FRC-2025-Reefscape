// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;

public class AlignToReef extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private boolean isLeftSide;
  private int reefSide;
  private boolean mirrorPose;
  private Pose2d goalPose;
  private boolean end = false;
  private final Alert unknownSide = new Alert("Unknown reef side!", AlertType.kError);
  private final int kRedIDoffset = 5;

  // private PIDController xController = DrivetrainConstants.kXController;
  // private PIDController yController = DrivetrainConstants.kYController;
  // private PIDController headingController = DrivetrainConstants.kHeadingController;
  private PPHolonomicDriveController cont = new PPHolonomicDriveController(DrivetrainConstants.kTranslationConstants, DrivetrainConstants.kHeadingConstants);
  private PathPlannerTrajectoryState goal = new PathPlannerTrajectoryState();

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private StructTopic<Pose2d> topic = inst.getStructTopic("GoalPose", Pose2d.struct);
  private StructPublisher<Pose2d> pub = topic.publish();

  /**
   * Auto alignment for reef scoring
   * @param drivetrain the drivetrain subsystem
   * @param isLeftSide True if the robot should align to the left reef branch
   * @param reefSide which side of the reef the robot should align to, from 1-6. ID's start with the bottom right side and increase counterclockwise.
   * @param mirrorPose True if the pose should be mirrored for the Blue alliance.
   */
  public AlignToReef(CommandSwerveDrivetrain drivetrain, boolean isLeftSide, int reefSide, boolean mirrorPose) {
    this.drivetrain = drivetrain;
    this.isLeftSide = isLeftSide;
    this.reefSide = reefSide;
    this.mirrorPose = mirrorPose;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    unknownSide.set(false);
    AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(reefSide + kRedIDoffset).ifPresentOrElse(
      (pose) -> goalPose = pose.toPose2d().plus( isLeftSide? VisionConstants.kLeftTransform : VisionConstants.kRightTransform),
      ()->{end = true; unknownSide.set(true);});

    pub.set(goalPose);
  }

  @Override
  public void execute() {
    goal.pose = goalPose;
    var speeds = cont.calculateRobotRelativeSpeeds(drivetrain.getState().Pose, goal);
    
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
