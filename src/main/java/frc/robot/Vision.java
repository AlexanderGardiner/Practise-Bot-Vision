package frc.robot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


public class Vision {
    public static Vision INSTANCE;

    public static Vision getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Vision();
        }   
        
        return INSTANCE;
    }

    private final PhotonCamera tagCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

    private PhotonPipelineResult pipeLast;

    public void periodic() {
        pipeLast = tagCam.getLatestResult();
        System.out.println("Best Target:" + this.getBestTarget());
    }

    public PhotonTrackedTarget getBestTarget(){
        if(pipeLast.hasTargets()){
            return pipeLast.getBestTarget();
        }
        else{
            return null;
        }
    }
    public boolean pipeLastHasTarget(){
        if(pipeLast == null){
            return false;
        }
        else{
            return true;
        }
    }
    
}
