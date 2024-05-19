package frc.robot.Subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kSwerve;
import monologue.Logged;
import monologue.Annotations.Log;
import frc.robot.Robot;

public class Swerve extends SubsystemBase implements Logged{

  public static Module FL = new Module(
    kSwerve.kMotorConstants.frontLeftSteer,
    kSwerve.kMotorConstants.frontLeftDrive,
    kSwerve.kAbsoluteEncoders.kFrontLeftDAbsoluteEncoderPort,
    kSwerve.kOffsets.FlOffset
  );
  public static Module FR = new Module(
    kSwerve.kMotorConstants.frontRightSteer,
    kSwerve.kMotorConstants.frontRightDrive,
    kSwerve.kAbsoluteEncoders.kFrontRightAbsoluteEncoderPort,
    kSwerve.kOffsets.FrOffset
  );
  public static Module BL = new Module(
    kSwerve.kMotorConstants.backLeftSteer,
    kSwerve.kMotorConstants.backLeftDrive,
    kSwerve.kAbsoluteEncoders.kBackLeftAbsoluteEncoderPort,
    kSwerve.kOffsets.BlOffset
  );
  public static Module BR = new Module(
    kSwerve.kMotorConstants.backRightSteer,
    kSwerve.kMotorConstants.backRightDrive,
    kSwerve.kAbsoluteEncoders.kBackRightAbsoluteEncoderPort,
    kSwerve.kOffsets.BrOffset
  );

  public SwerveDrivePoseEstimator PoseEstimator;

  public PIDController AngularPID;

  public Swerve() {
    PoseEstimator =
      new SwerveDrivePoseEstimator(
        kSwerve.kinematics,
        getGyroAngle(),
        getPositions(),
        new Pose2d()
      );
    AngularPID = new PIDController(0, 0, 0);
    AngularPID.enableContinuousInput(-Math.PI, Math.PI);

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getChassisSpeeds,
      this::AutoDrive,
      kSwerve.kAuto.config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this
    );
  }

  @Log.NT
  public Pose2d getEstimatedPose(){
    return PoseEstimator.getEstimatedPosition();
  }

  public Rotation2d getGyroAngle() {
    return Robot.Gyro.getRotation2d();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kSwerve.kinematics.toChassisSpeeds(getStates());
  }

  public void AutoDrive(ChassisSpeeds Speeds){
    Speeds = ChassisSpeeds.discretize(Speeds, 0.05);
    SwerveModuleState[] TargetStates = kSwerve.kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(TargetStates, kSwerve.MaxSpeed);
    setStates(TargetStates);
  }

  public void Drive(Translation2d translations, double OmegaRadiansPerSec, boolean fieldOrentation) {

    ChassisSpeeds Speeds =
    fieldOrentation ? 
    ChassisSpeeds.fromFieldRelativeSpeeds(
    translations.getX() * kSwerve.MaxSpeed,
    translations.getY() * kSwerve.MaxSpeed, 
    OmegaRadiansPerSec, 
    Robot.Gyro.getRotation2d())
    : new ChassisSpeeds(
        translations.getX() * kSwerve.MaxSpeed, 
        translations.getY() * kSwerve.MaxSpeed, 
        OmegaRadiansPerSec);

    Speeds = ChassisSpeeds.discretize(Speeds, 0.05);
    
    SwerveModuleState[] TargetStates = kSwerve.kinematics.toSwerveModuleStates(Speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(TargetStates, kSwerve.MaxSpeed);
    setStates(TargetStates);
  }

  public void resetPose(Pose2d pose) {
    PoseEstimator.resetPosition(getGyroAngle(), getPositions(), pose);
  }

  public Pose2d getPose() {
    return PoseEstimator.getEstimatedPosition();
  }

  public void setStates(SwerveModuleState[] states) {
    FL.setState(states[0]);
    FR.setState(states[1]);
    BL.setState(states[2]);
    BR.setState(states[3]);
  }

  @Log.NT
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      FL.getPosition(),
      FR.getPosition(),
      BL.getPosition(),
      BR.getPosition(),
    };
  }

  @Log.NT
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      FL.getState(),
      FR.getState(),
      BL.getState(),
      BR.getState()
    };
  }

  @Log.NT
  public SwerveModuleState[] getTargetStates(){
    return new SwerveModuleState[]
    {
      FL.getTargetState(),
      FR.getTargetState(),
      BL.getTargetState(),
      BR.getTargetState()
    };
  }
  
  @Log.NT
  public SwerveModuleState[] getOptimizedStates(){
    return new SwerveModuleState[]
    {
      FL.getOptimizedState(),
      FR.getOptimizedState(),
      BL.getOptimizedState(),
      BR.getOptimizedState()
    };
  }
}