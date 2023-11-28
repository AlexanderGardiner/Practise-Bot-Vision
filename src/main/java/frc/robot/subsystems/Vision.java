package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
    AprilTagFieldLayout aprilTagFieldLayout = null;
    PhotonPoseEstimator photonPoseEstimator = null;

    private PhotonPipelineResult currentResults;

    public Pose3d currentPose = new Pose3d();

    public Vision() {
        try {
            Transform3d robotToCam = new Transform3d(new Translation3d(13, 9, 8), new Rotation3d(0, 0, 0));
            aprilTagFieldLayout = loadFieldLayout();
    
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.LOWEST_AMBIGUITY, tagCam, robotToCam);
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    @Override
    public void periodic() {
        try {
            if (tagCam.getLatestResult().hasTargets()) {
                currentResults = tagCam.getLatestResult();
            } else {
                currentResults = null;
            }
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    @Override
    public void run() {
        while(true) {
            try {
                if (this.getEstimatedGlobalPose(currentPose.toPose2d()).isPresent()) {
                    currentPose = getEstimatedGlobalPose(currentPose.toPose2d()).get().estimatedPose;
                }
            } catch (Exception e) {
            // TODO: handle exception
            }            
        }
    }

    public Pose3d getCurrentPose() {
        return currentPose;
    }

    public PhotonTrackedTarget getBestTarget() {
        try {
            if (tagCam.getLatestResult().hasTargets()) {
                return currentResults.getBestTarget();
            }
        } catch (Exception e) {
            // TODO: handle exception
        }

        return null;
    }

    public boolean camHasTarget() {
        if (currentResults == null) {
            return false;
        } else {
            return true;
        }
    }

    private AprilTagFieldLayout loadFieldLayout() {
        try {
            return AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }
        return new AprilTagFieldLayout(null, 0, 0);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        } catch (Exception e) {
            // TODO: handle exception
        }
        return photonPoseEstimator.update();
    }

}
