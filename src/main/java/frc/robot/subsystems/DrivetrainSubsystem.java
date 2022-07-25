// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
//import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.ADIS16470_IMU;


public class DrivetrainSubsystem extends SubsystemBase {
  public static final double MAX_VOLTAGE = 12.0;
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Front right
    new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back left
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
    // Back right
    new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
);
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public ADIS16470_IMU gyro = new ADIS16470_IMU();
  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD,
    // This is the ID of the drive motor
    Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
    // This is the ID of the steer motor
    Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
    // This is the ID of the steer encoder
    Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
    // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
    Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    m_frontRightModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD,
    Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
    Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
    Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
    Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD,
    Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
    Constants.BACK_LEFT_MODULE_STEER_MOTOR,
    Constants.BACK_LEFT_MODULE_STEER_ENCODER,
    Constants.BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk3SwerveModuleHelper.createFalcon500(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),Mk3SwerveModuleHelper.GearRatio.STANDARD,
    Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
    Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
    Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
    Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  public void zeroGyroscope() {
    gyro.reset();
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    // This method will be called once per scheduler run
  }
}
