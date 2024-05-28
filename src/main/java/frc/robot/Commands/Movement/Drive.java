package frc.robot.Commands.Movement;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.kSwerve;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.Swerve;

public class Drive extends Command{


    private double xTranslation;
    private double yTranslation;
    private double OmegaRadiansPerSecond;
    private double AngleTranslation;
    private double Theta;
    private boolean fieldOrentation;
    private boolean slow;
    private boolean AngleControl;
    private boolean LimeLightTargeting;
    public Swerve swerve;
    
    public Drive(
    DoubleSupplier xTrans,
    DoubleSupplier yTrans,
    DoubleSupplier OmegaRadiansPerSec,
    DoubleSupplier AngTrans,
    BooleanSupplier fieldOrentation,
    BooleanSupplier Slow,
    BooleanSupplier AngleControl,
    BooleanSupplier LimeLightTargeting,
    Swerve swerve
    ){
        this.xTranslation = xTrans.getAsDouble();
        this.yTranslation = yTrans.getAsDouble();
        this.OmegaRadiansPerSecond = OmegaRadiansPerSec.getAsDouble();
        this.AngleTranslation = AngTrans.getAsDouble();
        this.slow = Slow.getAsBoolean();
        this.fieldOrentation = fieldOrentation.getAsBoolean();
        this.AngleControl = AngleControl.getAsBoolean();
        this.LimeLightTargeting = LimeLightTargeting.getAsBoolean();
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        
        Theta = Math.atan2(AngleTranslation, OmegaRadiansPerSecond);

        // Adds the dead area to controller input ex: input = 0.01 output = 0, mitigates drift/small movements
        xTranslation = MathUtil.applyDeadband(xTranslation, kSwerve.kControlConstants.kDeadband);
        yTranslation = MathUtil.applyDeadband(yTranslation, kSwerve.kControlConstants.kDeadband);
        OmegaRadiansPerSecond = MathUtil.applyDeadband(OmegaRadiansPerSecond, kSwerve.kControlConstants.kDeadband);
        
        // adds curve to controller input ex: 0.5 * 0.5 = 0.25
        xTranslation = Math.copySign(xTranslation * xTranslation, xTranslation);
        yTranslation = Math.copySign(yTranslation * yTranslation, yTranslation);
        OmegaRadiansPerSecond = Math.copySign(OmegaRadiansPerSecond * OmegaRadiansPerSecond, OmegaRadiansPerSecond);
        
        // get the rotation going max speed before MODs 
        OmegaRadiansPerSecond *= kSwerve.MaxSpeed;

        // reduces the value of OmegaRadiansPerSecond 
        if (slow) {
            OmegaRadiansPerSecond *= kSwerve.kSpeedMods.slowMod;
        }

        // track a Note on the field (currently color based)
        if (LimeLightTargeting){
           OmegaRadiansPerSecond = LimelightPropotionalAim();
        }

        // redefines OmegaRadiansPerSecond if controling the angle of the robot 
        if(AngleControl){
            OmegaRadiansPerSecond = swerve.AngularPID.calculate(Robot.Gyro.getRotation2d().getRadians(), Theta);
        }

        swerve.Drive(
            new Translation2d(xTranslation, yTranslation),
            OmegaRadiansPerSecond,
            fieldOrentation
        );
    }

    public double LimelightPropotionalAim(){
        // its the propotional (requires tuning)
        double kp = 0.001;

        // get the desired velo
        double TargetingVelo = LimelightHelpers.getTX("") * kp;

        // make it go fast
        TargetingVelo *= kSwerve.MaxSpeed;

        // invert because of Limelight
        TargetingVelo *= -1;

        // return our velocity
        return TargetingVelo;
    }
}