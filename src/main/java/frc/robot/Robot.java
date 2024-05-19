// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.Movement.Drive;
import frc.robot.Constants.kDrivers;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class Robot extends TimedRobot implements Logged{

  private Command m_autonomousCommand;

  // Needed hardware

  public static AHRS Gyro = new AHRS(Port.kMXP);

  // Subsystems
  public static Swerve mSwerve = new Swerve();
  public static Shooter mShooter = new Shooter();

  // autos

  public static SendableChooser<Command> autoChooser;

  // controllers

  public static Joystick Driver = new Joystick(0);
  public static Joystick Operator = new Joystick(1);

  @Override
  public void robotInit() {
    Monologue.setupMonologue(this, "Robot", false, false);
    
    mSwerve.setDefaultCommand(
      new Drive(
        () ->
          -Driver.getRawAxis(kDrivers.kcontrollerConstants.kAxisConstants.kLeftX),
        () ->
          -Driver.getRawAxis(kDrivers.kcontrollerConstants.kAxisConstants.kLeftY),
        () ->
          Driver.getRawAxis(kDrivers.kcontrollerConstants.kAxisConstants.kRightX),
        () ->
          Driver.getRawAxis(kDrivers.kcontrollerConstants.kAxisConstants.kRightY),
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
        mSwerve
      )
    );

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("None");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Monologue.setFileOnly(DriverStation.isFMSAttached());

    Monologue.updateAll();


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

    new POVButton(Driver, kDrivers.kcontrollerConstants.kButtonConstants.POV_UP)
      .whileTrue(Commands.print("up"));
    new POVButton(Driver, kDrivers.kcontrollerConstants.kButtonConstants.POV_DOWN)
      .whileTrue(Commands.print("Down"));
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
