// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.Settings;
import frc.robot.util.TrajectoryCache;
import frc.robot.util.TrajectoryCache.PATHTYPE;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final AHRS _gyro = new AHRS(SPI.Port.kMXP);
  private final Drive_Train _drive_Train = new Drive_Train(_gyro);
  private final Joystick _driver = new Joystick(0);
  private final Joystick _operator = new Joystick(1);
  private final Intake _intake = new Intake(); 
  private final ClimberHooks _climberHooks = new ClimberHooks();
  private final ClimberArm _climberArm = new ClimberArm();
  private final IntakeActuator _intakeActuator = new IntakeActuator();

  private SendableChooser<String> _pathChooser = new SendableChooser<String>(); 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    _drive_Train.setDefaultCommand(new ArcadeDrive(_drive_Train, _driver));
    _climberArm.setDefaultCommand(new ClimberArmDance(_climberArm, _operator));

    DriverStation.silenceJoystickConnectionWarning(true);
    
    loadPathChooser();
  }  

  private void loadPathChooser() {
    TrajectoryCache.clear();
    _pathChooser = new SendableChooser<>();

    // cacheTrajectory("Left", "paths/output/left.wpilib.json");
    // cacheTrajectory("Middle", "paths/output/middle.wpilib.json");
    // cacheTrajectory("Right", "paths/output/right.wpilib.json");
    // cacheTrajectory("Middle -> Left", "paths/output/middle_then_left.wpilib.json");
    
    cacheTrajectory("Middle", "middle", PATHTYPE.PathPlanner);
    cacheTrajectory("Left", "left", PATHTYPE.PathPlanner);    
    cacheTrajectory("Right", "right", PATHTYPE.PathPlanner);
    cacheTrajectory("Middle then Left", "middle then left", PATHTYPE.PathPlanner);
    cacheTrajectory("Middle then Right", "middle then right", PATHTYPE.PathPlanner);
    cacheTrajectory("Right then Middle", "right then middle", PATHTYPE.PathPlanner);
    cacheTrajectory("Test Straight", "test-straight", PATHTYPE.PathPlanner);
    cacheTrajectory("Another Test", "another-test", PATHTYPE.PathPlanner);

    SmartDashboard.putData("Path Chooser", _pathChooser);
  }

  private void cacheTrajectory(String key, String value, PATHTYPE type) {
    _pathChooser.addOption(key, key);
    if(type == PATHTYPE.PathWeaver)
      TrajectoryCache.addPathWeaver(key, value);
    else if(type == PATHTYPE.PathPlanner)
      TrajectoryCache.addPathPlanner(key, value, 1, 1);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(_driver, JoystickConstants.BUMPER_LEFT).whenPressed(new IntakeActuateOut(_intakeActuator));
    new JoystickButton(_driver, JoystickConstants.LOGO_LEFT).whenPressed(new IntakeActuateIn(_intakeActuator));
    new JoystickButton(_driver, JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeIn(_intake));
    new JoystickButton(_driver, JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeOut(_intake));
    new JoystickButton(_driver, JoystickConstants.X).whenPressed(new ClimberHooksForward(_climberHooks));
    new JoystickButton(_driver, JoystickConstants.B).whenPressed(new ClimberHooksBack(_climberHooks));
    
    new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT).whenPressed(new IntakeActuateOut(_intakeActuator));
    new JoystickButton(_operator, JoystickConstants.LOGO_LEFT).whenPressed(new IntakeActuateIn(_intakeActuator));
    new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeIn(_intake));
    new JoystickButton(_operator, JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeOut(_intake));
   // new JoystickButton(_operator, JoystickConstants.A).whileHeld(new ClimberArmDown(_climberArm));
   // new JoystickButton(_operator, JoystickConstants.Y).whileHeld(new ClimberArmUp(_climberArm));
    new JoystickButton(_operator, JoystickConstants.X).whenPressed(new ClimberHooksForward(_climberHooks));
    new JoystickButton(_operator, JoystickConstants.B).whenPressed(new ClimberHooksBack(_climberHooks));
    new JoystickButton(_operator, JoystickConstants.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(_climberArm::toggleSoftLimit, _climberArm));
    new JoystickButton(_operator, JoystickConstants.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(_climberArm::encoderReset, _climberArm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    String path = _pathChooser.getSelected();

    return new AutonDrivePath(_drive_Train, path);
  }

  public void loadSettings(){
    _intake.setPower(Settings.loadDouble("Intake", "Power", IntakeConstants.intakeMotorPower));
    _climberArm.setPower(Settings.loadDouble("ClimberArm", "Power", ClimberConstants.climberArmPower));
    _drive_Train.setksVolts(Settings.loadDouble("DriveTrain", "ksVolts", DrivetrainConstants.ksVolts));
    _drive_Train.setkvVoltSecondsPerMeter(Settings.loadDouble("DriveTrain", "kvVoltSecondsPerMeter", DrivetrainConstants.kvVoltSecondsPerMeter));
    _drive_Train.setkaVoltSecondsSquaredPerMeter(Settings.loadDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
    _drive_Train.setkPDriveVel(Settings.loadDouble("DriveTrain", "kPDriveVel", DrivetrainConstants.kPDriveVel));
  }  

  public void saveSettings(){
    Settings.saveDouble("Intake", "Power", _intake.getPower());
    Settings.saveDouble("ClimberArm", "Power", _climberArm.getPower());
    Settings.saveDouble("DriveTrain", "ksVolts", _drive_Train.getksVolts());
    Settings.saveDouble("DriveTrain", "kvVoltsSecondsPerMeter", _drive_Train.getkvVoltSecondsPerMeter());
    Settings.saveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", _drive_Train.getkaVoltSecondsSquaredPerMeter());
    Settings.saveDouble("DriveTrain", "kPDriveVel", _drive_Train.getkPDriveVel()); 
  }
}
