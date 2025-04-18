// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Comparator;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;

public class AlignedIntake extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private PIDController controller = new PIDController(0.5, 0, 0.02);
  private double angle = 0.0;
  private SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

  public AlignedIntake(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    controller.enableContinuousInput(-180, 180);
    SmartDashboard.putData(controller);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // var bestYaw = 1000;
    // var bestArea = 2800;
    if (Vision.latestCoralResult.hasTargets()){

      Optional<Pose2d> pose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(Vision.latestCoralResult.getTimestampSeconds()));
      if (pose.isPresent()){
        Optional<PhotonTrackedTarget> coralTarget = Vision.latestCoralResult.getTargets().stream()
          .filter(target->target.objDetectId==1)
          .min(Comparator.comparingDouble(target -> 5-Math.abs(target.yaw)));

        if (coralTarget.isPresent()){
          angle = pose.get().getRotation().getDegrees() -
           coralTarget.get().yaw;
          controller.setSetpoint(angle);
        }

        //var target:Vision.latestCoralResult.getTargets()  
        // while (Vision.latestCoralResult.getBestTarget().getDetectedObjectClassID() == 0)
        //   Vision.latestCoralResult.getTargets().remove(Vision.latestCoralResult.getBestTarget().)
        // }
      }
    }
    double calculated = controller.calculate(drivetrain.getState().Pose.getRotation().getDegrees());
    drivetrain.setControl(req.withSpeeds(new ChassisSpeeds(-2, 0, calculated)));
    SmartDashboard.putNumber("calculated", calculated);
    SmartDashboard.putNumber("robot rotation", drivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("setpoint", controller.getSetpoint());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
