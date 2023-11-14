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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
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

    public Vision() {
        Transform3d robotToCam = new Transform3d(new Translation3d(13, 9, 8), new Rotation3d(0, 0, 0));
        aprilTagFieldLayout = loadFieldLayout();

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.LOWEST_AMBIGUITY, tagCam, robotToCam);
    }

    @Override
    public void periodic() {
        if (tagCam.getLatestResult().hasTargets()) {
            currentResults = tagCam.getLatestResult();
        } else {
            currentResults = null;
        }
    }

    public PhotonTrackedTarget getBestTarget() {
        if (tagCam.getLatestResult().hasTargets()) {
            return currentResults.getBestTarget();
        } else {
            return null;
        }
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
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

}
