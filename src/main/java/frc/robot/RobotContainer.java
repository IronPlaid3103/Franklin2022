// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.Settings;
import frc.robot.util.TrajectoryCache;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Climber _climber = new Climber();

  private SendableChooser<String> _pathChooser = new SendableChooser<String>(); 
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    _drive_Train.setDefaultCommand(new ArcadeDrive(_drive_Train, _driver)); 
    
    loadPathChooser();
  }  

  private void loadPathChooser() {
    TrajectoryCache.clear();
    _pathChooser = new SendableChooser<>();

    //cacheTrajectory("GS A Red", "Paths/output/GS_A--Red.wpilib.json");

    cacheTrajectory("Test-Straight", "Paths/output/test-straight.wpilib.json");
    cacheTrajectory("Test-CurveLeft", "Paths/output/test-curveleft.wpilib.json");
    
    //_pathChooser.addOption("Test-Group", "Test-Group");

    SmartDashboard.putData("Path Chooser", _pathChooser);
  }

  private void cacheTrajectory(String key, String trajectoryJson) {
    _pathChooser.addOption(key, key);
    TrajectoryCache.add(key, trajectoryJson);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //OPERATOR
    new JoystickButton(_driver, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeActuateOut(_intake));
    new JoystickButton(_driver, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeActuateIn(_intake));
    new JoystickButton(_driver, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeIn(_intake));
    new JoystickButton(_driver, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeOut(_intake));
    new JoystickButton(_driver, Constants.JoystickConstants.A).whileHeld(new ClimberMotorDown(_climber));
    new JoystickButton(_driver, Constants.JoystickConstants.Y).whileHeld(new ClimberMotorUp(_climber));
    new JoystickButton(_driver, Constants.JoystickConstants.X).whileHeld(new ClimberSolenoidForward(_climber));
    new JoystickButton(_driver, Constants.JoystickConstants.B).whileHeld(new ClimberSolenoidBack(_climber));

    //DRIVER
    // *** All of these buttons are for ease of testing during development - recommend commenting out before competition build and deploy ***
    new JoystickButton(_operator, Constants.JoystickConstants.BUMPER_LEFT).whileHeld(new IntakeActuateOut(_intake));
    new JoystickButton(_operator, Constants.JoystickConstants.LOGO_LEFT).whileHeld(new IntakeActuateIn(_intake));
    new JoystickButton(_operator, Constants.JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeIn(_intake));
    new JoystickButton(_operator, Constants.JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeOut(_intake));
    new JoystickButton(_operator, Constants.JoystickConstants.A).whileHeld(new ClimberMotorDown(_climber));
    new JoystickButton(_operator, Constants.JoystickConstants.Y).whileHeld(new ClimberMotorUp(_climber));
    new JoystickButton(_operator, Constants.JoystickConstants.X).whileHeld(new ClimberSolenoidForward(_climber));
    new JoystickButton(_operator, Constants.JoystickConstants.B).whileHeld(new ClimberSolenoidBack(_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    String path = _pathChooser.getSelected();
    return new AutonIntake(_drive_Train, _intake, path);
  }

  public void loadSettings(){
    _intake.setPower(Settings.loadDouble("Intake", "Power", IntakeConstants.intakeMotorPower));
    _climber.setMotorPower(Settings.loadDouble("Climber", "MotorPower", ClimberConstants.climberMotorPower));
    _drive_Train.setksVolts(Settings.loadDouble("DriveTrain", "ksVolts", DrivetrainConstants.ksVolts));
    _drive_Train.setkvVoltSecondsPerMeter(Settings.loadDouble("DriveTrain", "kvVoltSecondsPerMeter", DrivetrainConstants.kvVoltSecondsPerMeter));
    _drive_Train.setkaVoltSecondsSquaredPerMeter(Settings.loadDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
  }  

  public void saveSettings(){
    Settings.saveDouble("Intake", "Power", _intake.getPower());
    Settings.saveDouble("Climber", "MotorPower", _climber.getMotorPower());
    Settings.saveDouble("DriveTrain", "ksVolts", _drive_Train.getksVolts());
    Settings.saveDouble("DriveTrain", "kvVoltsSecondsPerMeter", _drive_Train.getkvVoltSecondsPerMeter());
    Settings.saveDouble("DriveTrain", "kaVoltSecondsSquaredPerMeter", _drive_Train.getkaVoltSecondsSquaredPerMeter());
  }
}
