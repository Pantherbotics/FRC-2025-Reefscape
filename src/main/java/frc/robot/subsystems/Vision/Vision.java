/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot.subsystems.Vision; 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
 import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 
 public class Vision {

    private final StructArrayPublisher<Pose3d> estPub = NetworkTableInstance.getDefault().getTable("Pose").getStructArrayTopic("EstimatedPoses", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> leftTargetPub = NetworkTableInstance.getDefault().getTable("Pose").getSubTable("VisionTargets").getStructArrayTopic("left vision targets", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> rightTargetPub = NetworkTableInstance.getDefault().getTable("Pose").getSubTable("VisionTargets").getStructArrayTopic("right vision targets", Pose3d.struct).publish();

    private final PhotonCamera leftCam = new PhotonCamera(VisionConstants.kLeftCamName);
    private final PhotonCamera rightCam = new PhotonCamera(VisionConstants.kRightCamName);

     private final PhotonPoseEstimator leftEstimator;
     private final PhotonPoseEstimator rightEstimator;
     private Matrix<N3, N1> curStdDevs;
 
     // Simulation
     private PhotonCameraSim leftSim;
     private PhotonCameraSim rightSim;
     private VisionSystemSim visionSim;
     public Vision() {

         leftEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLeftCamTransform);
         leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
         rightEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRightCamTransform);
         rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation

         if (Robot.isSimulation()) {
            
            // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(VisionConstants.kAprilTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.4, 0.10);
             cameraProp.setFPS(60);
             cameraProp.setAvgLatencyMs(30);
             cameraProp.setLatencyStdDevMs(10);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.

             leftSim = new PhotonCameraSim(leftCam, cameraProp);
             rightSim = new PhotonCameraSim(rightCam, cameraProp);
             visionSim.addCamera(leftSim, VisionConstants.kRobotToLeftCamTransform);
             visionSim.addCamera(rightSim, VisionConstants.kRobotToRightCamTransform);
             leftSim.enableDrawWireframe(true);
             rightSim.enableDrawWireframe(true);
             // Add the simulated camera to view the targets on this simulated field.
             
 
         }
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator estimator) {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
             for (var change : camera.getAllUnreadResults()) {
                 visionEst = estimator.update(change);
                 updateEstimationStdDevs(visionEst, change.getTargets());
     
                 if (Robot.isSimulation()) {
                     visionEst.ifPresentOrElse(
                             est ->
                                     getSimDebugField()
                                             .getObject("VisionEstimation" + camera.getName())
                                             .setPose(est.estimatedPose.toPose2d()),
                             () -> {
                                 getSimDebugField().getObject("VisionEstimation").setPoses();
                             });
                 }
             }
         return visionEst;
     }

     public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses(){
        var estPoses = List.of(getEstimatedGlobalPose(leftCam, leftEstimator), getEstimatedGlobalPose(rightCam, rightEstimator));
        Pose3d[] poses = new Pose3d[2];
        for (int i = 0; i < estPoses.size(); i++) {
            poses[i] = estPoses.get(i).map(p -> p.estimatedPose).orElse(Pose3d.kZero); // Use null or a default instance
        }
        estPub.set(poses);
        var targetPoses = extractTagPoses(estPoses);
        leftTargetPub.set(targetPoses[0]);
        rightTargetPub.set(targetPoses[1]);
        return estPoses;
     }

     public static Pose3d[][] extractTagPoses(List<Optional<EstimatedRobotPose>> estimatedPoses) {
        Pose3d[][] tagPosesArray = new Pose3d[estimatedPoses.size()][];

        for (int i = 0; i < estimatedPoses.size(); i++) {
            Optional<EstimatedRobotPose> estimatedPoseOpt = estimatedPoses.get(i);
            List<Pose3d> tagPoses = new ArrayList<>();

            if (estimatedPoseOpt.isPresent()) {
                EstimatedRobotPose estimatedPose = estimatedPoseOpt.get();

                for (PhotonTrackedTarget target : estimatedPose.targetsUsed) {
                    Optional<Pose3d> tagPoseOpt = VisionConstants.kAprilTagLayout.getTagPose(target.getFiducialId());
                    tagPoseOpt.ifPresent(tagPoses::add);
                }
            }

            // Convert List<Pose3d> to Pose3d[] and store it in the array
            tagPosesArray[i] = tagPoses.toArray(new Pose3d[0]);
        }

        return tagPosesArray;
    }

 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     public void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = VisionConstants.kSingleTagStdDevs;
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = VisionConstants.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = leftEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 curStdDevs = VisionConstants.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 25));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPoses()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
         return curStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }
 }