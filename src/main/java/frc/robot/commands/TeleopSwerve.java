package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private BooleanSupplier turnToAngleSup;
    private BooleanSupplier slowMode;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    
    

    private boolean lockRotate;

    public TeleopSwerve(Swerve s_Swerve,
     DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,
     BooleanSupplier turnToAngle, BooleanSupplier slowMode) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        this.turnToAngleSup = turnToAngle;
        this.slowMode = slowMode;

        translationLimiter = new SlewRateLimiter(Constants.Swerve.slewRate);
        strafeLimiter = new SlewRateLimiter(Constants.Swerve.slewRate);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        double rotationVal;

        if(turnToAngleSup.getAsBoolean()){
            lockRotate = true;
        }


        if (lockRotate && Math.abs(rotationSup.getAsDouble()) < Constants.stickDeadband){
            rotationVal = s_Swerve.calculateSwerveRotation(180) + Constants.Swerve.angleKF;
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxRotationalSpeed, Constants.Swerve.maxRotationalSpeed);
        } else {
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        }


        if(slowMode.getAsBoolean()) {
            translationVal *= 0.5;
            strafeVal *= 0.5;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationLimiter.calculate(translationVal),
            strafeLimiter.calculate(strafeVal))
             .times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            true, 
            false
        );
    }
}