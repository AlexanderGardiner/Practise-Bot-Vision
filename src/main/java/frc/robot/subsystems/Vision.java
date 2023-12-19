package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.HashMap;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase implements Runnable {
    public static Vision INSTANCE;

    public static Vision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Vision();
        }

        return INSTANCE;
    }

    private final PhotonCamera tagCam = new PhotonCamera("Octocam_2");
    private final PhotonCamera tagCam2 = new PhotonCamera("Octocam_3");
    AprilTagFieldLayout aprilTagFieldLayout = null;
    PhotonPoseEstimator photonPoseEstimatorOne = null;
    PhotonPoseEstimator photonPoseEstimatorTwo = null;

    private List<PhotonTrackedTarget> currentResults;

    private HashMap<Integer, PhotonTrackedTarget> currentHashMap = new HashMap<Integer, PhotonTrackedTarget>();

    public Pose3d currentPoseCamOne = new Pose3d();
    public Pose3d currentPoseCamTwo = new Pose3d();

    public Vision() {
        try {
            Transform3d robotToCamOne = new Transform3d(
                    new Translation3d(Units.inchesToMeters(11), Units.inchesToMeters(11), Units.inchesToMeters(17)),
                    new Rotation3d(0, 0, 0));
            Transform3d robotToCamTwo = new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11), Units.inchesToMeters(10), Units.inchesToMeters(17)),
                    new Rotation3d(0, 0, 0));
            aprilTagFieldLayout = loadFieldLayout();

            photonPoseEstimatorOne = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY, tagCam, robotToCamOne);
            photonPoseEstimatorTwo = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY, tagCam2, robotToCamTwo);
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    @Override
    public void periodic() {
        try {
            // if (tagCam.getLatestResult().hasTargets()) {
            //     currentResults = tagCam.getLatestResult().getTargets();
            // }

            currentResults = tagCam.getLatestResult().hasTargets() ? tagCam.getLatestResult().getTargets() : null;

            for (int i = 0; i < currentResults.size() - 1; i++) {
                currentHashMap.put(currentResults.get(i).getFiducialId(), currentResults.get(i));
            }

            // if (tagCam2.getLatestResult().hasTargets()) {
            //     for (int i = 0; i < tagCam2.getLatestResult().getTargets().size() - 1; i++) {
            //         currentResults.add(tagCam2.getLatestResult().getTargets().get(i));
            //     }
            // }

            //#region
            //Version 1
            // currentResults.addAll(tagCam2.getLatestResult().hasTargets() ? tagCam2.getLatestResult().getTargets() : null);

            //Version 2
            // if (tagCam2.getLatestResult().hasTargets()) {
            //     for (int i = 0; i < currentResults.size(); i++) {
            //         for (int j = 0; j < tagCam2.getLatestResult().getTargets().size() - 1; j++) {
            //             if (currentResults.get(i).getFiducialId() == tagCam2.getLatestResult().getTargets().get(j)
            //                     .getFiducialId()) {
            //                 currentResults.set(i, tagCam2.getLatestResult().getTargets().get(j));
            //             } else {
            //                 currentResults.add(tagCam2.getLatestResult().getTargets().get(j));
            //             }
            //         }
            //     }
            // }

            //Version 3
            // currentResults = currentResults.stream().map((PhotonTrackedTarget target) -> {
            //     return tagCam2.getLatestResult().getTargets().stream()
            //             .filter(newTarget -> target.getFiducialId() == newTarget.getFiducialId())
            //             .findFirst()
            //             .orElse(target);
            // }).collect(Collectors.toList());

            //Version 4 -- Needs Fixes
            // currentResults = currentResults.stream().map((PhotonTrackedTarget target) -> {
            //     // Find the matching target from the other list
            //     PhotonTrackedTarget foundTarget = tagCam2.getLatestResult().getTargets().stream().findAny(
            //             (PhotonTrackedTarget newTarget) -> target.getFiducialId() == newTarget.getFiducialId())
            //             .orElse(null);

            //     // Return the modified or original target
            //     return foundTarget != null ? foundTarget : target;
            // }).collect(Collectors.toList());

            //#endregion

            //Version 5
            if (tagCam2.getLatestResult().hasTargets()) {
                for (int i = 0; i < tagCam2.getLatestResult().getTargets().size() - 1; i++) {
                    currentHashMap.put(tagCam2.getLatestResult().getTargets().get(i).getFiducialId(),
                            tagCam2.getLatestResult().getTargets().get(i));
                }
            }

            currentResults = currentHashMap.values().stream().collect(Collectors.toList());

        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    //-------------Understandable version of run function below--------------------
    //#region
    // @Override
    // public void run() {
    //     while (true) {
    //         try {

    //             if (!currentResults.isEmpty()) {
    //                 // Iterator<PhotonTrackedTarget> iterator = currentResults.iterator();
    //                 for (int i = 0; i < currentResults.size() - 1; i++) {
    //                     currentResults.remove(
    //                             currentResults.get(i).getPoseAmbiguity() > 0.5 ? currentResults.get(i) : null);
    //                     i--;
    //                 }
    //             }
    //             // if (this.getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d()).isPresent()) {
    //             currentPoseCamOne = this.getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d()).isPresent()
    //                     ? null
    //                     : getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d()).get().estimatedPose;
    //             // }

    //             // if (this.getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d()).isPresent()) {
    //             currentPoseCamTwo = this.getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d()).isPresent()
    //                     ? null
    //                     : getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d()).get().estimatedPose;
    //             // }
    //         } catch (Exception e) {
    //             // TODO: handle exception
    //         }
    //     }
    // }
    //#endregion

    @Override
    public void run() {
        while (true) {
            try {
                // if (!currentResults.isEmpty()) {
                currentResults.removeIf(target -> !currentResults.isEmpty() && target.getPoseAmbiguity() > 0.2);
                // }

                getEstimatedGlobalPoseOne(currentPoseCamOne.toPose2d())
                        .ifPresent(target -> currentPoseCamOne = target.estimatedPose);

                getEstimatedGlobalPoseTwo(currentPoseCamTwo.toPose2d())
                        .ifPresent(target -> currentPoseCamTwo = target.estimatedPose);

                // Optional<EstimatedRobotPose> estimatedPoseCamOne = getEstimatedGlobalPoseOne(
                //         currentPoseCamOne.toPose2d());

                // if (estimatedPoseCamOne.isPresent()) {
                //     currentPoseCamOne = estimatedPoseCamOne.get().estimatedPose;
                // }

                // Optional<EstimatedRobotPose> estimatedPoseCamTwo = getEstimatedGlobalPoseTwo(
                //         currentPoseCamTwo.toPose2d());
                // if (estimatedPoseCamTwo.isPresent()) {
                //     currentPoseCamTwo = estimatedPoseCamTwo.get().estimatedPose;
                // }
            } catch (Exception e) {
                //Nothing
            }
        }
    }

    public Pose3d getCurrentPoseOne() {
        return currentPoseCamOne;
    }

    public Pose3d getCurrentPoseTwo() {
        return currentPoseCamTwo;
    }

    // public PhotonTrackedTarget getBestTarget() {
    //     try {
    //         if (tagCam.getLatestResult().hasTargets()) {
    //             return currentResults.getBestTarget();
    //         }
    //     } catch (Exception e) {
    //         // TODO: handle exception
    //     }

    //     return null;
    // }

    public boolean camHasTarget() {
        return currentResults != null;
        // if (currentResults == null) {
        //     return false;
        // } else {
        //     return true;
        // }
    }

    private AprilTagFieldLayout loadFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        return new AprilTagFieldLayout(null, 0, 0);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseOne(Pose2d prevEstimatedRobotPose) {
        try {
            photonPoseEstimatorOne.setReferencePose(prevEstimatedRobotPose);
        } catch (Exception e) {
            // TODO: handle exception
        }
        return photonPoseEstimatorOne.update();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseTwo(Pose2d prevEstimatedRobotPose) {
        try {
            photonPoseEstimatorTwo.setReferencePose(prevEstimatedRobotPose);
        } catch (Exception e) {
            // TODO: handle exception
        }
        return photonPoseEstimatorTwo.update();
    }

}
