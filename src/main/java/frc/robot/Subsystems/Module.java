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
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;

public class Module extends SubsystemBase {

  private Rotation2d Offset;

  private SwerveModuleState State;
  private SwerveModuleState optimizedState;

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
    optimizedState = new SwerveModuleState();

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
    optimizedState = SwerveModuleState.optimize(state, getAngle());

    DriveMotor.setVoltage(OpenLoopFF.calculate(optimizedState.speedMetersPerSecond));

    if (state.speedMetersPerSecond < kSwerve.MaxSpeed * 0.01) {
      AzimuthMotor.setVoltage(0);
    } else {
      AzimuthMotor.setVoltage(
        AzimuthController.calculate(
          optimizedState.angle.getRotations(),
          getAngle().getRotations()
        )
      );
    }
  }


  public SwerveModuleState getTargetState(){
    return State;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(gearReduction() , getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(RPM_TO_M_per_S(DriveEncoder.getVelocity()), getAngle());
  }

  public SwerveModuleState getOptimizedState(){
    return new SwerveModuleState(optimizedState.speedMetersPerSecond, optimizedState.angle);
  }

  // Converts RPM to M/S for any module
  private double RPM_TO_M_per_S(double RPM){
    double velocity = (((2 * Math.PI) * (Constants.kSwerve.wheelDiameter /2 )) / 60) * RPM;
    return velocity;
  }

  // for SDS L1 module use 8.14, use 6.75 and 6.12 for L2 and L3 respectively 
  private double gearReduction(){
    return DriveEncoder.getPosition() / ((Constants.kSwerve.wheelDiameter * Math.PI) / 8.14);
  }
}
