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
import frc.robot.util.XboxPOV;
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
  public final LEDStrip led = new LEDStrip(100, 0);
  private final Drive_Train _drive_Train = new Drive_Train(_gyro, led);
  private final Joystick _driver = new Joystick(0);
  private final Joystick _operator = new Joystick(1);
  // private final Intake _intake = new Intake(); 
  // private final IntakeActuator _intakeActuator = new IntakeActuator();
  private final ClimberHooks _climberHooks = new ClimberHooks();
  private final ClimberArm _climberArm = new ClimberArm();
  private SendableChooser<String> _pathChooser = new SendableChooser<String>(); 
  private final Camera _camera = new Camera(); 
  
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
    
    cacheTrajectory("Straight Forward", "straight forward", PATHTYPE.PathPlanner);
    cacheTrajectory("Straight Backward", "straight backward", PATHTYPE.PathPlanner);
    cacheTrajectory("Cargo - Left", "cargo - left", PATHTYPE.PathPlanner);    
    cacheTrajectory("Cargo - Middle", "cargo - middle", PATHTYPE.PathPlanner);
    cacheTrajectory("Cargo - Right", "cargo - right", PATHTYPE.PathPlanner);
    cacheTrajectory("Cargo - Middle then Left", "cargo - middle then left", PATHTYPE.PathPlanner);
    cacheTrajectory("Cargo - Middle then Right", "cargo - middle then right", PATHTYPE.PathPlanner);
    cacheTrajectory("Cargo - Right then Middle", "cargo - right then middle", PATHTYPE.PathPlanner);
    
    // cacheTrajectory("Test Straight", "cargo - test-straight", PATHTYPE.PathPlanner);

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
    //DRIVER
    new JoystickButton(_driver, JoystickConstants.BUMPER_LEFT).whenPressed(new InstantCommand(_camera::toggleCamera, _camera)); 
    new JoystickButton(_driver, JoystickConstants.BUMPER_RIGHT).whenPressed(new InstantCommand(_camera::toggleCamera, _camera)); 

    // new JoystickButton(_driver, JoystickConstants.BUMPER_LEFT).whenPressed(new IntakeActuateOut(_intakeActuator));
    // new JoystickButton(_driver, JoystickConstants.LOGO_LEFT).whenPressed(new IntakeActuateIn(_intakeActuator));
    // new JoystickButton(_driver, JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeIn(_intake));
    // new JoystickButton(_driver, JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeOut(_intake));
    // new JoystickButton(_driver, JoystickConstants.X).whenPressed(new ClimberHooksForward(_climberHooks));
    // new JoystickButton(_driver, JoystickConstants.B).whenPressed(new ClimberHooksBack(_climberHooks));

    //OPERATOR
    new JoystickButton(_operator, JoystickConstants.B).whenPressed(new ClimberHooksForward(_climberHooks));
    new JoystickButton(_operator, JoystickConstants.X).whenPressed(new ClimberHooksBack(_climberHooks));
    new JoystickButton(_operator, JoystickConstants.LOGO_LEFT).whenPressed(new InstantCommand(_climberArm::toggleSoftLimit, _climberArm));
    new JoystickButton(_operator, JoystickConstants.LOGO_RIGHT).whenPressed(new InstantCommand(_climberArm::encoderReset, _climberArm));

    new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT).whenPressed(new InstantCommand(_camera::toggleCamera, _camera)); 
    new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT).whenPressed(new InstantCommand(_camera::toggleCamera, _camera)); 


    // new JoystickButton(_operator, JoystickConstants.A).whileHeld(new ClimberArmDown(_climberArm));
    // new JoystickButton(_operator, JoystickConstants.Y).whileHeld(new ClimberArmUp(_climberArm));
    // new XboxPOV(_operator).whenPressed(new ClimberAutoClimb(_climberArm, _climberHooks, _intakeActuator));

    // new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT).whileHeld(new IntakeCargo(_intake, _intakeActuator));
    // new JoystickButton(_operator, JoystickConstants.BUMPER_RIGHT).whenReleased(new IntakeActuateIn(_intakeActuator));
    // new JoystickButton(_operator, JoystickConstants.LOGO_RIGHT).whileHeld(new IntakeIn(_intake));
    // new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT).whileHeld(new VomitCargo(_intake, _intakeActuator));
    // new JoystickButton(_operator, JoystickConstants.BUMPER_LEFT).whenReleased(new IntakeActuateIn(_intakeActuator));
    // new JoystickButton(_operator, JoystickConstants.LOGO_LEFT).whileHeld(new IntakeOut(_intake));
    // new XboxPOV(_operator).whenPressed(new IntakeActuateOut(_intakeActuator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    String path = _pathChooser.getSelected();

    return new AutonDrivePathWithWait(_drive_Train, path, _climberHooks);
  }

  public void loadSettings(){
    // _intake.setPower(Settings.loadDouble("Intake", "Power", IntakeConstants.intakeMotorPower));
    _climberArm.setPower(Settings.loadDouble("Arm", "Power", ClimberConstants.climberArmPower));
    _drive_Train.setksVolts(Settings.loadDouble("DriveTrain", "ks", DrivetrainConstants.ksVolts));
    _drive_Train.setkvVoltSecondsPerMeter(Settings.loadDouble("DriveTrain", "kv", DrivetrainConstants.kvVoltSecondsPerMeter));
    _drive_Train.setkaVoltSecondsSquaredPerMeter(Settings.loadDouble("DriveTrain", "ka", DrivetrainConstants.kaVoltSecondsSquaredPerMeter));
    _drive_Train.setkPDriveVel(Settings.loadDouble("DriveTrain", "kp", DrivetrainConstants.kPDriveVel));
    _climberArm.setLimit(Settings.loadDouble("Arm", "Top Limit", ClimberConstants.REVERSE_LIMIT));

    SmartDashboard.putNumber("Auton Wait", 0);
  }  

  public void saveSettings(){
    // Settings.saveDouble("Intake", "Power", _intake.getPower());
    Settings.saveDouble("Arm", "Power", _climberArm.getPower());
    Settings.saveDouble("DriveTrain", "ks", _drive_Train.getksVolts());
    Settings.saveDouble("DriveTrain", "kv", _drive_Train.getkvVoltSecondsPerMeter());
    Settings.saveDouble("DriveTrain", "ka", _drive_Train.getkaVoltSecondsSquaredPerMeter());
    Settings.saveDouble("DriveTrain", "kp", _drive_Train.getkPDriveVel()); 
    Settings.saveDouble("Arm", "Top", _climberArm.getLimit()); 
  }
}
