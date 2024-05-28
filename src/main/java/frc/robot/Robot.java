// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.Movement.Drive;
import frc.robot.Constants.kDrivers;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged {

  private Command m_autonomousCommand;

  // Needed hardware

  public static AHRS Gyro = new AHRS(Port.kMXP);

  // Subsystems
  public static Swerve mSwerve = new Swerve();
  public static Shooter mShooter = new Shooter();
  public static Intake mIntake = new Intake();
  public static LimelightHelpers mLimeLight = new LimelightHelpers();

  // autos

  public static SendableChooser<Command> autoChooser;

  // controllers

  public static Joystick Driver = new Joystick(0);
  public static Joystick Operator = new Joystick(1);

  @Override
  public void robotInit() {
    Monologue.setupMonologue(this, "Robot", false, false);

    // Camera Pose can be set this way which allows for movement of the camera through a match eg. (BumbleB 2024)
    // this can also be set in the WebUI for limelight, this is best for our use case

    // LimelightHelpers.setCameraPose_RobotSpace("",
    //  kDefaultPeriod,
    //  kDefaultPeriod,
    //  kDefaultPeriod,
    //  kDefaultPeriod,
    //  kDefaultPeriod,
    //  kDefaultPeriod);

    mSwerve.setDefaultCommand(
      new Drive(
        () ->
          -Driver.getRawAxis(
            kDrivers.kcontrollerConstants.kAxisConstants.kLeftX
          ),
        () ->
          -Driver.getRawAxis(
            kDrivers.kcontrollerConstants.kAxisConstants.kLeftY
          ),
        () ->
          Driver.getRawAxis(
            kDrivers.kcontrollerConstants.kAxisConstants.kRightX
          ),
        () ->
          Driver.getRawAxis(
            kDrivers.kcontrollerConstants.kAxisConstants.kRightY
          ),
        () ->
          !Driver.getRawButton(
            kDrivers.kcontrollerConstants.kButtonConstants.kStart
          ),
        () ->
          Driver.getRawButton(
            kDrivers.kcontrollerConstants.kButtonConstants.kRightBumper
          ),
        () ->
          Driver.getRawButton(
            kDrivers.kcontrollerConstants.kButtonConstants.kLeftBumper
          ),
        () ->
          Driver.getRawButton(
            kDrivers.kcontrollerConstants.kButtonConstants.kA
          ),
        mSwerve
      )
    );

    configureBindings();
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser("None");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Monologue.setFileOnly(DriverStation.isFMSAttached());
    Monologue.updateAll();

    boolean Reject = false;
    // seting the orentation of the robot based on gyro mesurements
    // used for megatag 2
    LimelightHelpers.SetRobotOrientation(
      "",
      Gyro.getYaw(),
      Gyro.getRate(),
      Gyro.getPitch(),
      0,
      Gyro.getRoll(),
      0
    );

    // for using Vision in pose estimation it requires
    // The pose of the limelight must be configured via the API or the webUI
    // a field map has been uploaded (.fmap) this is provided by FIRST
    // a Bot pose estimate is made per periodic cycle
    // Set robot orentaton is has a blue corner origin

    // creates a BotPose est
    LimelightHelpers.PoseEstimate MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
      ""
    );

    // if the rate of rotation is above 720 degrees per second, regect the pose
    if (Math.abs(Gyro.getRate()) > 720) {
      Reject = true;
    }
    // if no tags, reject the pose
    if (MegaTag2.tagCount == 0) {
      Reject = true;
    }
    // if the pose was not rejected set the values to trust in pose est and update the pose
    if (!Reject) {
      mSwerve.PoseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(0.7, 0.7, 99999999)
      );
      mSwerve.PoseEstimator.addVisionMeasurement(
        MegaTag2.pose,
        MegaTag2.timestampSeconds
      );
    }
  }

  private void configureBindings() {
    new JoystickButton(
      Operator,
      kDrivers.kcontrollerConstants.kAxisConstants.kRightTrigger
    )
      .toggleOnTrue(mShooter.ShootingMode())
      .toggleOnFalse(mShooter.IdleMode());
    new JoystickButton(
      Operator,
      kDrivers.kcontrollerConstants.kButtonConstants.kLeftBumper
    )
      .toggleOnTrue(mShooter.AmpMode())
      .toggleOnFalse(mShooter.IdleMode());
  }

  public void registerNamedCommands() {
    NamedCommands.registerCommand("ShooterFull", mShooter.ShootingMode());
    NamedCommands.registerCommand("ShooterIdle", mShooter.IdleMode());
    NamedCommands.registerCommand(
      "IntakeDown",
      new InstantCommand(() -> mIntake.IntakeDown())
    );
    NamedCommands.registerCommand(
      "IntakeUp",
      new InstantCommand(() -> mIntake.IntakeUp())
    );
    NamedCommands.registerCommand(
      "IntakeSpit",
      new InstantCommand(() -> mIntake.IntakeSpit())
    );
    NamedCommands.registerCommand(
      "IntakeFeed",
      new InstantCommand(() -> mIntake.FeedShooter())
    );
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
