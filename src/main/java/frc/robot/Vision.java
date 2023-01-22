package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision {
    PhotonCamera m_Camera;
    PhotonPipelineResult m_Result;

    public Vision() {
        m_Camera = new PhotonCamera("OV5647");
    }

    public void processVision() {
        m_Result = m_Camera.getLatestResult();
        System.out.println("Ran process vision");
        System.out.println("Camera is Connected: " + m_Camera.isConnected());
    }

    public boolean isTargetPresent() {
        System.out.println("Has Target: " +  m_Result.hasTargets());
        return m_Result.hasTargets();
    }

    public double getTargetYaw() {
        PhotonTrackedTarget target = m_Result.getBestTarget();
        return target.getYaw();
    }
}
