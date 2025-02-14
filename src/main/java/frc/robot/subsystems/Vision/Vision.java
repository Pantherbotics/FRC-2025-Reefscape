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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class Vision {
    private static final PhotonCamera m_leftCamera = new PhotonCamera(VisionConstants.kLeftCamName);
    private static final PhotonCamera m_rightCamera = new PhotonCamera(VisionConstants.kRightCamName);
    private static final PhotonPoseEstimator m_leftEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLeftCamTransform);
    private static final PhotonPoseEstimator m_rightEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRightCamTransform);
    public static PhotonPipelineResult m_leftResult;
    public static PhotonPipelineResult m_rightResult;
    public static Optional<EstimatedRobotPose> leftEstimatedPose;
    public static Optional<EstimatedRobotPose> rightEstimatedPose;
    public static Optional<EstimatedRobotPose>[] estimatedPoses;

    public static void setup(){
        m_leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        m_rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static void updateCamera(){
        m_leftResult = m_leftCamera.getLatestResult();
        leftEstimatedPose = m_leftEstimator.update(m_leftResult);
        m_rightResult = m_rightCamera.getLatestResult();
        rightEstimatedPose = m_rightEstimator.update(m_rightResult);

        if (leftEstimatedPose.isPresent()){
            double[]pose = new double[3];
            pose[0] = leftEstimatedPose.get().estimatedPose.getX();
            pose[1] = leftEstimatedPose.get().estimatedPose.getY();
            pose[2] = leftEstimatedPose.get().estimatedPose.getRotation().getAngle();
            SmartDashboard.putNumberArray("pose", pose);
        }
        

    } 
    
}
