package frc.robot.Subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber.kPorts;
import frc.robot.Constants.kClimber.kSpeeds;

public class Climber extends SubsystemBase {

  private CANSparkMax LeftClimber = new CANSparkMax(
    kPorts.LeftPort,
    MotorType.kBrushless
  );
  private CANSparkMax RightClimber = new CANSparkMax(
    kPorts.RightPort,
    MotorType.kBrushless
  );

  public Climber() {
    LeftClimber.setIdleMode(IdleMode.kBrake);
    LeftClimber.setInverted(false);
    LeftClimber.enableVoltageCompensation(12);
    LeftClimber.setSmartCurrentLimit(20);

    RightClimber.setIdleMode(IdleMode.kBrake);
    RightClimber.setInverted(false);
    RightClimber.enableVoltageCompensation(12);
    LeftClimber.setSmartCurrentLimit(20);
  }

  // Climbers will go down
  public void ClimbersDown() {
    RightClimber.setVoltage(kSpeeds.WindDown);
    LeftClimber.setVoltage(kSpeeds.WindDown);
  }

  // Climbers will go up
  public void ClimbersUP() {
    RightClimber.setVoltage(kSpeeds.WindUp);
    LeftClimber.setVoltage(kSpeeds.WindUp);
  }

  // Change the idle mode of the climbers
  public void ClimbersChangeIdleMode(IdleMode mode) {
    RightClimber.setIdleMode(mode);
    LeftClimber.setIdleMode(mode);
  }
}
