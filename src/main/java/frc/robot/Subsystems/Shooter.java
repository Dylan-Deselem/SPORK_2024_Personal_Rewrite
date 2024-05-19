package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {

  public enum modes {
    Shooting,
    Amp,
    Idle,
  }

  private modes mode;

  private CANSparkMax LNEO = new CANSparkMax(
    kShooter.kPorts.LeftNeoPort,
    MotorType.kBrushless
  );
  private CANSparkMax RNEO = new CANSparkMax(
    kShooter.kPorts.RightNeoPort,
    MotorType.kBrushless
  );

  private double DesiredSpeed;

  private PIDController SpeedsController;

  public Shooter() {
    LNEO.enableVoltageCompensation(kShooter.kLimits.NominalVoltage);
    LNEO.setSmartCurrentLimit(kShooter.kLimits.CurrentLimit);

    RNEO.enableVoltageCompensation(kShooter.kLimits.NominalVoltage);
    RNEO.setSmartCurrentLimit(20);

    SpeedsController =
      new PIDController(
        kShooter.kShooterPID.kP,
        kShooter.kShooterPID.kI,
        kShooter.kShooterPID.kD
      );

    mode = modes.Idle;
  }

  @Override
  public void periodic() {
    switch (mode) {
      case Shooting:
        DesiredSpeed = kShooter.kSpeeds.MaxSpeed;
        break;
      case Amp:
        DesiredSpeed = kShooter.kSpeeds.AmpSpeed;
        break;
      case Idle:
        DesiredSpeed = kShooter.kSpeeds.IdleSpeed;
        break;
      default:
        DesiredSpeed = 0;
        break;
    }

    LNEO.setVoltage(
      SpeedsController.calculate(DesiredSpeed, LNEO.getBusVoltage())
    );
    RNEO.setVoltage(
      SpeedsController.calculate(DesiredSpeed, RNEO.getBusVoltage())
    );
  }

  public Command ShootingMode() {
    return this.runOnce(() -> mode = modes.Shooting);
  }

  public Command AmpMode() {
    return this.runOnce(() -> mode = modes.Amp);
  }

  public Command IdleMode() {
    return this.runOnce(() -> mode = modes.Idle);
  }
}
