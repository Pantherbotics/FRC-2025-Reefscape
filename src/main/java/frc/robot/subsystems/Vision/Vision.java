// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.visionConstants;

/** Add your docs here. */
public class Vision {
    private static PhotonCamera camera = new PhotonCamera("PC_Camera");
    private static PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.LOWEST_AMBIGUITY, visionConstants.kRobotToMainCamTransform);
    public static PhotonPipelineResult result;
    public static Optional<EstimatedRobotPose> estimatedPose;

    public static void setup(){
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static void updateCamera(){
        result = camera.getLatestResult();
        estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isPresent()){
            double[]pose = new double[3];
            pose[0] = estimatedPose.get().estimatedPose.getX();
            pose[1] = estimatedPose.get().estimatedPose.getY();
            pose[2] = estimatedPose.get().estimatedPose.getRotation().getAngle();
            SmartDashboard.putNumberArray("pose", pose);
        }
        

    } 
    
}
