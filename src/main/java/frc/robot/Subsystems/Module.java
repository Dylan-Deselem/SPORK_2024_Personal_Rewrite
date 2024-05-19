package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;

public class Module extends SubsystemBase {

  private Rotation2d Offset;

  private SwerveModuleState State;

  public CANcoder AbsoluteEncoder;
  public CANcoderConfiguration config;
  public CANcoderConfigurator CANconfig;

  private CANSparkMax DriveMotor;
  private CANSparkMax AzimuthMotor;

  private PIDController AzimuthController;

  private RelativeEncoder DriveEncoder;

  private SimpleMotorFeedforward OpenLoopFF = new SimpleMotorFeedforward(
    0,
    0,
    0
  );

  public Module(
    int DriveID,
    int AzimuthID,
    int AbsoluteEncoderID,
    Rotation2d Offset
  ) {
    this.Offset = Offset;
    State = new SwerveModuleState();

    DriveMotor = new CANSparkMax(DriveID, MotorType.kBrushless);
    DriveEncoder = DriveMotor.getEncoder();
    DriveEncoder.setPosition(0);
    DriveMotor.setIdleMode(kSwerve.kMotorConstants.DriveIdleMode);
    DriveMotor.setInverted(false);

    AzimuthMotor = new CANSparkMax(AzimuthID, MotorType.kBrushless);
    AzimuthMotor.setIdleMode(kSwerve.kMotorConstants.AzumuthIdleMode);
    AzimuthMotor.setInverted(false);

    AzimuthController = new PIDController(0, 0, 0);

    AzimuthController.enableContinuousInput(0, 1);

    AbsoluteEncoder = new CANcoder(AbsoluteEncoderID);
    CANconfig = AbsoluteEncoder.getConfigurator();
    config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange =
      AbsoluteSensorRangeValue.Unsigned_0To1;
    CANconfig.apply(config);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
      AbsoluteEncoder.getPosition().getValueAsDouble() - Offset.getRotations()
    );
  }

  public void setState(SwerveModuleState state) {
    State = state;
    DriveMotor.setVoltage(OpenLoopFF.calculate(state.speedMetersPerSecond));

    if (state.speedMetersPerSecond < kSwerve.MaxSpeed * 0.01) {
      AzimuthMotor.setVoltage(0);
    } else {
      AzimuthMotor.setVoltage(
        AzimuthController.calculate(
          state.angle.getRotations(),
          getAngle().getRotations()
        )
      );
    }
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(DriveEncoder.getPosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(State.speedMetersPerSecond, getAngle());
  }
}
