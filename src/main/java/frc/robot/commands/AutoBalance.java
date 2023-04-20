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
        /* Get Values, Deadband */
        double driveSpeed;
        final double maxRoll = 15; //Increase - real max value is 18

        double roll = s_Swerve.getRoll() / maxRoll;

        roll = MathUtil.clamp(roll, -1, 1);
        if (Math.abs(s_Swerve.getRoll()) > 18) {
            driveSpeed = (Math.signum(roll) * roll * roll) * .5;
        }
        else if (Math.abs(s_Swerve.getRoll()) > 1.8) {
            driveSpeed = (Math.signum(roll) * roll * roll) * .22; // Cube instead of square?
        } else {
            driveSpeed = 0;
        }

        System.out.println("Raw roll " + s_Swerve.getRoll());
        System.out.println("Calculated: " + roll);

        // if(roll > 5){
        // driveSpeed = .13;
        // } else if(roll < -5){
        // driveSpeed = -.13;
        // } else {driveSpeed = 0;}

        /* Drive */
        s_Swerve.drive(
                new Translation2d(driveSpeed * Constants.Swerve.maxSpeed, 0),
                0,
                false,
                true);
    }
}