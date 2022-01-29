// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.JoystickConstants;

public class Drive_Train extends SubsystemBase {
  
  private final CANSparkMax _fLMotor = new CANSparkMax(DrivetrainConstants.frontLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _fRMotor = new CANSparkMax(DrivetrainConstants.frontRightMotor, MotorType.kBrushless);
  private final CANSparkMax _bLMotor = new CANSparkMax(DrivetrainConstants.rearLeftMotor, MotorType.kBrushless);
  private final CANSparkMax _bRMotor = new CANSparkMax(DrivetrainConstants.rearRightMotor, MotorType.kBrushless);

  private final DifferentialDrive _drive = new DifferentialDrive(_fLMotor, _fRMotor); 

  private AHRS _gyro;

  private RelativeEncoder _left_follower;
  private RelativeEncoder _right_follower;

  private DifferentialDriveOdometry _odometry;

  private Pose2d _pose;

  private double _ksVolts = DrivetrainConstants.ksVolts;
  private double _kvVoltSecondsPerMeter = DrivetrainConstants.kvVoltSecondsPerMeter;
  private double _kaVoltSecondsSquaredPerMeter = DrivetrainConstants.kaVoltSecondsSquaredPerMeter;

  public Drive_Train(AHRS gyro) {  

    _gyro = gyro;
    _gyro.reset();
    
    _fLMotor.restoreFactoryDefaults();
    _fRMotor.restoreFactoryDefaults();
    _bLMotor.restoreFactoryDefaults();
    _bRMotor.restoreFactoryDefaults();

    _fLMotor.setInverted(true);
    _bLMotor.setInverted(true);

    _bLMotor.follow(_fLMotor);
    _bRMotor.follow(_fRMotor);

    enableOpenLoopRampRate(true);

    _left_follower = _fLMotor.getEncoder();
    _right_follower = _fRMotor.getEncoder();

    _left_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);
    _right_follower.setPositionConversionFactor(DrivetrainConstants.kDistancePerWheelRevolutionMeters / DrivetrainConstants.kGearReduction);

    encoderReset();
    
    _odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  public void enableOpenLoopRampRate(boolean enable) {
    double rampRate = (enable ? DrivetrainConstants.rampRate : 0.0);

    _fLMotor.setOpenLoopRampRate(rampRate);
    _fRMotor.setOpenLoopRampRate(rampRate);
    _bLMotor.setOpenLoopRampRate(rampRate);
    _bRMotor.setOpenLoopRampRate(rampRate);
  }

  public void teleopDrive(Joystick driveControl) {
    double forward = applyDeadband(driveControl.getRawAxis(JoystickConstants.LEFT_STICK_Y));
    double turn = applyDeadband(driveControl.getRawAxis(JoystickConstants.RIGHT_STICK_X));

    _drive.arcadeDrive(forward, -turn);
  }

  private double applyDeadband(double value) {
    if(Math.abs(value) < DrivetrainConstants.deadband)
      return 0.0;
    else
      return (value - Math.copySign(.1, value)) / (1 - DrivetrainConstants.deadband);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    _pose = _odometry.update(_gyro.getRotation2d(), _left_follower.getPosition(), _right_follower.getPosition());
  }

  public void encoderReset() {
    _right_follower.setPosition(0.0);
    _left_follower.setPosition(0.0);
  }

  public Pose2d getPose() {
    return _pose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_left_follower.getVelocity() / 60, _right_follower.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    encoderReset();
    _odometry.resetPosition(pose, _gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _fLMotor.setVoltage(leftVolts);
    _fRMotor.setVoltage(-rightVolts);
    _bRMotor.setVoltage(-rightVolts);
    _bLMotor.setVoltage(leftVolts);
  }

  public double getkaVoltSecondsSquaredPerMeter() {
    return _kaVoltSecondsSquaredPerMeter;
  }

  public double getkvVoltSecondsPerMeter() {
    return _kvVoltSecondsPerMeter;
  }

  public double getksVolts(){
    return _ksVolts;
  }

  public void setkaVoltSecondsSquaredPerMeter(double kaVoltSecondsSquaredPerMeter) {
    _kaVoltSecondsSquaredPerMeter = kaVoltSecondsSquaredPerMeter;
  }

  public void setkvVoltSecondsPerMeter(double kvVoltSecondsPerMeter) {
    _kvVoltSecondsPerMeter = kvVoltSecondsPerMeter;
  }

  public void setksVolts(double ksVolts){
    _ksVolts = ksVolts;
  }
}
