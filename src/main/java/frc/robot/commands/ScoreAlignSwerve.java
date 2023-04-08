package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.autos.DriveAUto;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ScoreAlignSwerve extends CommandBase {    
    private Swerve s_Swerve;  
    private Vision s_Vision; 
    private double goalYaw = .7;
    private boolean drive;
    
    public ScoreAlignSwerve(Swerve s_Swerve, Vision s_Vision) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.s_Vision = s_Vision;
        drive = false;
    }

    @Override
    public void initialize(){
        drive = false;
    }

    @Override
    public void execute() {
        double rotationVal;
        

        
            rotationVal = s_Swerve.calculateSwerveRotation(180) + Constants.Swerve.angleKF;
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxRotationalSpeed, Constants.Swerve.maxRotationalSpeed);
        
            double translationVal = 0;
            double strafeVal = 0;

            s_Vision.processVision();

            if(s_Vision.isTargetPresent()) {
                double targetYaw = s_Vision.getHighYaw() + 1;
                boolean inRange = Math.abs(targetYaw) < goalYaw;
                System.out.println("IN RANGE: "+inRange);
                

                strafeVal = s_Vision.calculateStrafe(targetYaw, inRange) + (Math.signum(-targetYaw) * .08);
                if (inRange){
                    drive = true;
                }
                if (drive) {
                        translationVal = .23;
                        strafeVal = 0;
                }
            }


        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal)
             .times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }
}