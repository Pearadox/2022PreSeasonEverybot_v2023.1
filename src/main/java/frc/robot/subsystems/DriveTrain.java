// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class DriveTrain extends SubsystemBase {
  private CANSparkMax _frontLeft = new CANSparkMax(12, MotorType.kBrushless);
  private CANSparkMax _backLeft = new CANSparkMax(13, MotorType.kBrushless);
  private CANSparkMax _frontRight = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax _backRight = new CANSparkMax(10, MotorType.kBrushless);
  private DifferentialDrive _drive = new DifferentialDrive(_frontLeft, _frontRight);
  private ADIS16470_IMU _gyro;
  private DifferentialDriveOdometry _odometry;
  private RelativeEncoder _frontLeftEncoder;
  private RelativeEncoder _frontRightEncoder;
  private RelativeEncoder _backLeftEncoder;
  private RelativeEncoder _backRightEncoder;

  public DriveTrain(ADIS16470_IMU gyro) {
    SendableRegistry.add(_drive, "drive");

    _frontLeft.restoreFactoryDefaults();
    _backLeft.restoreFactoryDefaults();
    _frontRight.restoreFactoryDefaults();
    _backRight.restoreFactoryDefaults();

    _backLeft.follow(_frontLeft);
    _backRight.follow(_frontRight);

    _frontLeft.burnFlash();
    _backLeft.burnFlash();
    _frontRight.burnFlash();
    _backRight.burnFlash();

    _gyro = gyro;

    _frontLeftEncoder = _frontLeft.getEncoder();
    _frontRightEncoder = _frontRight.getEncoder();
    _backLeftEncoder = _backLeft.getEncoder();
    _backRightEncoder = _backRight.getEncoder();

    _frontLeftEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters / DriveTrainConstants.kGearReduction);
    _frontRightEncoder.setPositionConversionFactor(DriveTrainConstants.kDistancePerWheelRevolutionMeters / DriveTrainConstants.kGearReduction);

    resetEncoders();
    _odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(_gyro.getAngle()), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    _odometry.update(Rotation2d.fromDegrees(_gyro.getAngle()), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition());

    SmartDashboard.putNumber("Gyro", _gyro.getAngle());
    SmartDashboard.putNumber("frontLeftEnc", _frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("frontRightEnc", -_frontRightEncoder.getPosition());
    SmartDashboard.putNumber("backRightEnc", -_backRightEncoder.getPosition());
    SmartDashboard.putNumber("backLeftEnc", _backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Pose X", _odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose y", _odometry.getPoseMeters().getY());
  }

  public void teleopDrive(Joystick controller) {
    double axis4 = controller.getRawAxis(4);
    double axis1 = controller.getRawAxis(1);
    drive(axis4, axis1);
  }

  public void drive(double rotation, double direction) {
    _drive.arcadeDrive(rotation, direction);
  }

  public RelativeEncoder getEncoder() {
    return _frontLeft.getEncoder();
  }

  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_frontLeftEncoder.getVelocity() / 60, -_frontRightEncoder.getVelocity() / 60);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(Rotation2d.fromDegrees(_gyro.getAngle()), _frontLeftEncoder.getPosition(), -_frontRightEncoder.getPosition(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _frontLeft.setVoltage(leftVolts);
    _backLeft.setVoltage(leftVolts);
    _frontRight.setVoltage(-rightVolts);
    _backRight.setVoltage(-rightVolts);

    _drive.feed();
  }

  public void resetEncoders() {
    _frontLeftEncoder.setPosition(0);
    _frontRightEncoder.setPosition(0);
  }

  public void zeroHeading() {
    _gyro.reset();
  }
}