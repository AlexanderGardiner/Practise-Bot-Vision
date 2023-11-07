package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    public static Vision INSTANCE;

    public static Vision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Vision();
        }

        return INSTANCE;
    }

    private final PhotonCamera tagCam = new PhotonCamera("Octocam_2");

    private PhotonPipelineResult currentResults;

    @Override
    public void periodic() {
        if (tagCam.getLatestResult().hasTargets()) {
            currentResults = tagCam.getLatestResult();
        }
        else {
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

}
