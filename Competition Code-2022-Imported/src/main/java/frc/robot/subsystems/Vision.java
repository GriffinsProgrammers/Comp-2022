package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase{
    private PhotonCamera camera;
    private PhotonPipelineResult result;

    public Vision()
    {
        camera = new PhotonCamera("photonvision");
        result = camera.getLatestResult();
    }

    public double getYaw()
    {
        return result.getBestTarget().getYaw();
    }

    public boolean canSeeBall()
    {
        if(result.hasTargets())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public double getArea()
    {
        return result.getBestTarget().getArea();
    }
}
