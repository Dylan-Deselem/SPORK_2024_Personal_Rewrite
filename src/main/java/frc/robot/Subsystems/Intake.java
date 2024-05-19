package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

public class Intake extends SubsystemBase{
    
    public CANSparkMax PivotNEO = new CANSparkMax(kIntake.kPorts.PivotNeoPort, MotorType.kBrushless);
    public CANSparkMax IntakeNeoMini = new CANSparkMax(kIntake.kPorts.IntakeNeoPort, MotorType.kBrushless);
    
    public SparkPIDController IntakePID;

    public RelativeEncoder PivotEncoder;

    public Intake(){
        PivotNEO.enableVoltageCompensation(kIntake.kLimits.NominalVoltage);
        PivotNEO.setSmartCurrentLimit(kIntake.kLimits.CurrentLimit);
        PivotNEO.setIdleMode(IdleMode.kBrake);

        IntakeNeoMini.enableVoltageCompensation(kIntake.kLimits.NominalVoltage);
        IntakeNeoMini.setIdleMode(IdleMode.kCoast);

        IntakePID = PivotNEO.getPIDController();
        IntakePID.setP(kIntake.kPIDConstants.kP);
        IntakePID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        IntakePID.setSmartMotionMaxVelocity(kIntake.kLimits.MaxRPM, 0);

        PivotEncoder = PivotNEO.getEncoder();
    }

    public void FeedShooter(){
        IntakeNeoMini.setIdleMode(IdleMode.kCoast);
        IntakeNeoMini.set(kIntake.kSpeeds.FeedSpeed);
    }

    public void IntakeDown(){
        IntakePID.setReference(kIntake.kPositions.IntakePOS, ControlType.kPosition);
        IntakeNeoMini.setIdleMode(IdleMode.kBrake);
        IntakeNeoMini.set(kIntake.kSpeeds.IntakeSpeed);
    }

    public void IntakeUp(){
        IntakePID.setReference(kIntake.kPositions.IntakePOS, ControlType.kPosition);
        IntakeNeoMini.set(0);
    }

    public void IntakeSpit(){
        IntakePID.setReference(kIntake.kPositions.IntakePOS, ControlType.kPosition);
    }
}
