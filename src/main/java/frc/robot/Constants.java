// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public static final class DrivetrainConstants {
        public static final int frontLeftMotor = 1;
        public static final int frontRightMotor = 2;
        public static final int rearLeftMotor = 3;
        public static final int rearRightMotor = 4;  
        
        public static final double deadband = 0.1;

        // public static final double ksVolts = 0.146;//0.292
        // public static final double kvVoltSecondsPerMeter = 2.675;//0.2.85
        // public static final double kaVoltSecondsSquaredPerMeter = 0.2049;//0.0537

        // public static final double kTrackwidthMeters = Units.inchesToMeters(24);
        // public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // public static final double kPDriveVel = 0;//2.05;

        public static final double rampRate = 0.5;
		// public static final double kDistancePerWheelRevolutionMeters = Units.inchesToMeters(Math.PI * 6.0);//0.47877872
        // public static final double kGearReduction = 10.71;
        
        // //Galactic Search
        // public static final double kachunkPower = 0.2;
        // public static final double kachunkTime = 0.5;
        // public static final double driveRightkP = 0.005;
    }

    // public static final class AutoConstants{

	// 	public static final double kMaxSpeedMetersPerSecond = 0;
	// 	public static final double kMaxAccelerationMetersPerSecondSquared = 0;

    //     public static final double kRamseteB = 2;
    //     public static final double kRamseteZeta = 0.7;
    // }

    public static final class JoystickConstants {
        //Controllers
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
            
        //XboxOne Joysticks (axes)
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_STICK_X = 4;
        public static final int RIGHT_STICK_Y = 5;
        
        //XboxOne Buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int BUMPER_LEFT = 5;
        public static final int BUMPER_RIGHT = 6;
        public static final int LOGO_LEFT = 7;
        public static final int LOGO_RIGHT = 8;
        public static final int LEFT_STICK_BUTTON = 9;
        public static final int RIGHT_STICK_BUTTON = 10;
    }

    public static final class IntakeConstants {
        public static final int intakeMotor = 5;
        public static final double defaultPower = 0.5;
        public static final int actuatorMotor = 6;
    }

    public static final class ClimberConstants {
        public static final int climberMotor = 5;
        public static final int climberSolenoid = 6;
        public static final double climberMotorPower = 0.5;
    }
}
