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

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class Vision {
    private static final PhotonCamera m_leftCamera = new PhotonCamera(VisionConstants.kLeftCamName);
    private static final PhotonCamera m_rightCamera = new PhotonCamera(VisionConstants.kRightCamName);
    private static final PhotonPoseEstimator m_leftEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToLeftCamTransform);
    private static final PhotonPoseEstimator m_rightEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.LOWEST_AMBIGUITY, VisionConstants.kRobotToRightCamTransform);
    public static PhotonPipelineResult m_leftResult;
    public static PhotonPipelineResult m_rightResult;
    public static Optional<EstimatedRobotPose> leftEstimatedPose;
    public static Optional<EstimatedRobotPose> rightEstimatedPose;
    public static Pose3d[] estimatedPoses = new Pose3d[]{Pose3d.kZero, Pose3d.kZero};

    private final static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final static NetworkTable table = inst.getTable("Vision Estimations");
    private final static StructArrayTopic<Pose3d> topic = table.getStructArrayTopic("Poses", Pose3d.struct);
    private final static StructArrayPublisher<Pose3d> pub = topic.publish();

    public static void setup(){
        m_leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        m_rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public static void updateCamera(){
        m_leftResult = m_leftCamera.getLatestResult();
        leftEstimatedPose = m_leftEstimator.update(m_leftResult);
        m_rightResult = m_rightCamera.getLatestResult();
        rightEstimatedPose = m_rightEstimator.update(m_rightResult);
        leftEstimatedPose.ifPresent(
            (pose) -> estimatedPoses[0] = pose.estimatedPose);
        rightEstimatedPose.ifPresent(
            (pose) ->estimatedPoses[1] = pose.estimatedPose);

        pub.set(estimatedPoses);

    } 
    
}
