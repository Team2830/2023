package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmStates;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private Swerve s_Swerve;

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

    private BooleanSupplier turnToForwardSup;
    private BooleanSupplier turnToBackwardSup;
    private BooleanSupplier slowMode;

    private SlewRateLimiter translationLimiter;
    private SlewRateLimiter strafeLimiter;
    private SlewRateLimiter armStrafeLimiter;

    private boolean backwardButtonState = false;
    private boolean forwardButtonState = false;

    private boolean lockRotate = false;

    private double goalAngle = 0;
    private boolean wasArm = false;

    public TeleopSwerve(Swerve s_Swerve,
            DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup,
            BooleanSupplier turnToForwardSup, BooleanSupplier turnToBackwardSup, BooleanSupplier slowMode) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        this.turnToForwardSup = turnToForwardSup;
        this.turnToBackwardSup = turnToBackwardSup;

        this.slowMode = slowMode;

        translationLimiter = new SlewRateLimiter(Constants.Swerve.slewRate);
        strafeLimiter = new SlewRateLimiter(Constants.Swerve.slewRate);
        armStrafeLimiter = new SlewRateLimiter(Constants.Swerve.slowRate);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeInput = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * .8;

        double rotationVal;
        double strafeVal;

        if (turnToForwardSup.getAsBoolean()) {
            if (!forwardButtonState) {
                forwardButtonState = true;
                goalAngle = s_Swerve.calculateGoalAngle(0);
                lockRotate = true;
            }
        } else {
            forwardButtonState = false;
        }

        if (turnToBackwardSup.getAsBoolean()) {
            if (!backwardButtonState) {
                backwardButtonState = true;
                goalAngle = s_Swerve.calculateGoalAngle(180);
                lockRotate = true;
            }
        } else {
            backwardButtonState = false;
        }

        if (lockRotate && Math.abs(rotationSup.getAsDouble()) < Constants.stickDeadband) {
            rotationVal = s_Swerve.calculateSwerveRotation(goalAngle);
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxRotationalSpeed,
                    Constants.Swerve.maxRotationalSpeed);
        } else {
            lockRotate = false;
            rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) * .5 * .25;
        }

        if (slowMode.getAsBoolean()) {
            translationVal *= 0.4;
            strafeInput *= 0.4;
        }

        // if (RobotContainer.ArmState == ArmStates.MID || RobotContainer.ArmState ==
        // ArmStates.HIGH) {
        // if (!wasArm){
        // strafeVal = armStrafeLimiter.calculate(strafeLimiter.calculate(strafeInput));
        // }
        // else{
        // strafeVal = armStrafeLimiter.calculate(strafeInput);
        // }
        // wasArm = true;
        // }
        // else {
        // if (wasArm){
        // strafeVal = strafeLimiter.calculate(armStrafeLimiter.calculate(strafeInput));
        // }
        // else{
        // strafeVal = strafeLimiter.calculate(strafeInput);
        // }
        // wasArm = false;
        // }
        strafeVal = strafeLimiter.calculate(strafeInput);
        /* Drive */
        s_Swerve.drive(
                new Translation2d(translationLimiter.calculate(translationVal),
                        strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true,
                false);
    }
}