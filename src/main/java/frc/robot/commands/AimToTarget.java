package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AimToTarget extends CommandBase {    
    private Swerve s_Swerve;
    private Vision vision;

    public AimToTarget(Swerve s_Swerve, Vision vision) {
        this.s_Swerve = s_Swerve;
        this.vision = vision;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double rotationVal;

        vision.processVision();

        if(vision.isTargetPresent()) {
            if(vision.getTargetYaw() > 2){
                rotationVal = 0.2;
            } else if(vision.getTargetYaw() < -2){
            rotationVal = -0.2;
            } else {rotationVal = 0;}
            System.out.println("yaw: " + vision.getTargetYaw());
        } else{
            rotationVal = 0;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(0, 0), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
    }
}