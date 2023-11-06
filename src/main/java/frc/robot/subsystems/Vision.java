package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    public static Vision INSTANCE;

    public static Vision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Vision();
        }

        return INSTANCE;
    }

    private final PhotonCamera tagCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    private PhotonPipelineResult currentResults;

    public void periodic() {
        if (tagCam.getLatestResult().hasTargets()) {
            currentResults = tagCam.getLatestResult();
        }
        System.out.println("Best Target:" + this.getBestTarget());
    }

    public PhotonTrackedTarget getBestTarget() {
        if (currentResults.hasTargets()) {
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

}
