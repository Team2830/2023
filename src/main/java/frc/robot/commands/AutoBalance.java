package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Vision;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalance extends CommandBase {    
    private Swerve s_Swerve;

    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double driveSpeed;

        
        System.out.println("X Pitch: " + s_Swerve.getRoll());
        if(s_Swerve.getRoll() > 5){
            System.out.println("Drive forward");
                driveSpeed = .13;
            } else if(s_Swerve.getRoll() < -5){
            driveSpeed = -.13;
            } else {driveSpeed = 0;}

        /* Drive */
        s_Swerve.drive(
            new Translation2d(driveSpeed * Constants.Swerve.maxSpeed, 0), 
            0, 
            false, 
            true
        );
    }

    private static int getPitch() {
        return 0;
    }
}