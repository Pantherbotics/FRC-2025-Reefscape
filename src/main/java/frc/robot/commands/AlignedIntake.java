// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake.GroundIntakeRollers;
import frc.robot.subsystems.GroundIntake.GroundPivot;
import frc.robot.subsystems.Vision.Vision;

public class AlignedIntake extends Command {
  private final GroundIntakeRollers rollers;
  private final GroundPivot pivot;
  private final CommandSwerveDrivetrain drivetrain;
  private PIDController controller = new PIDController(1, 0, 0);
  private double angle = 0.0;
  private SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

  public AlignedIntake(GroundIntakeRollers rollers, GroundPivot pivot, CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.rollers = rollers;
    this.pivot = pivot; 
    addRequirements(drivetrain, rollers, pivot);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Vision.latestCoralResult.hasTargets()){
      Optional<Pose2d> pose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(Vision.latestCoralResult.getTimestampSeconds()));
      if (pose.isPresent()){    
        angle = pose.get().getRotation().getDegrees() + Vision.latestCoralResult.getBestTarget().yaw;
        controller.setSetpoint(angle);
      }
    }
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds(2, 0, Units.degreesToRadians(controller.calculate(angle) + 180))));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
