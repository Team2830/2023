package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;

public class Vision {
    PhotonCamera m_Camera;
    PhotonPipelineResult m_Result;

    public Vision() {
        m_Camera = new PhotonCamera("OV5647");
        m_Camera.setLED(VisionLEDMode.kOn);
    }

    public void processVision() {
        m_Result = m_Camera.getLatestResult();

       // System.out.println("Camera is Connected: " + m_Camera.isConnected());
    }

    public boolean isTargetPresent() {
        System.out.println("Has Target: " + m_Result.hasTargets());
        return m_Result.hasTargets();
    }

    public double getTargetYaw() {
        PhotonTrackedTarget target = m_Result.getBestTarget();
        return target.getYaw();
    }

    public double getHighYaw() {
        double highYaw = 0;
        PhotonTrackedTarget target = getHighTarget();
        if (target != null) {
            highYaw = target.getYaw();
        }

        return highYaw;
    }

    public double getHighArea() {
        double highArea = 0;
        PhotonTrackedTarget target = getHighTarget();
        if (target != null) {
            highArea = target.getArea();
        }

        return highArea;
    }

    private PhotonTrackedTarget getHighTarget() {
        PhotonTrackedTarget closest = null;

        double minYaw = 360;

        processVision();
        List<PhotonTrackedTarget> targets = m_Result.getTargets();

        if (targets != null && targets.size() > 0) {
            for (PhotonTrackedTarget target : targets) {

                if (target.getPitch() > 0 && Math.abs(target.getYaw()) < minYaw) {
                    closest = target;
                    minYaw = Math.abs(target.getYaw());
                }

            }
        }
        return closest;
    }

    public double calculateStrafe(double targetYaw, boolean inRange) {
        double strafeVal = 0;


        if (!inRange){
            strafeVal = (targetYaw) * Constants.Swerve.visionAlignConstant;
            strafeVal = MathUtil.clamp(strafeVal, -.3, .3);    
        }
       
        return strafeVal;
    }
}
