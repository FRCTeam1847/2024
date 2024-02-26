// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");

  private final CANSparkMax m_leftDrive1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive1 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax m_leftDrive2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);

  private final DifferentialDrive Drive = new DifferentialDrive(m_leftDrive1, m_rightDrive1);

  private final Encoder leftEncoder = new Encoder(2, 3);
  private final Encoder rightEncoder = new Encoder(1, 0);

  private static final double cpr = 360;
  private static final double whd = 6;

  private final WPI_PigeonIMU gyro = new WPI_PigeonIMU(0);

  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

  private Field2d m_field = new Field2d();

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    leftEncoder.setDistancePerPulse(Math.PI * (whd / cpr));
    rightEncoder.setDistancePerPulse(Math.PI * (whd / cpr));
    rightEncoder.setReverseDirection(true);
    leftEncoder.setReverseDirection(true);

    m_rightDrive1.setInverted(true);
    m_leftDrive2.follow(m_leftDrive1);
    m_rightDrive2.follow(m_rightDrive1);
    leftEncoder.reset();
    rightEncoder.reset();
    SmartDashboard.putData("Field", m_field);

  }

  /** Update robot odometry. */
  public void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    m_odometry.resetPosition(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), pose);
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void ArcadeDrive(double X, double Z) {
    Drive.arcadeDrive(X, Z);
  }

  // public Command getPosition(){}

  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> Drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command driveDistanceCommand(double distanceInches, double speed) {
    return runOnce(
        () -> {
          // Reset encoders at the start of the command
          leftEncoder.reset();
          rightEncoder.reset();
        })
        // Drive forward at specified speed
        .andThen(run(() -> {
          double angle = gyro.getAngle();
          ArcadeDrive(speed, 0);
        }))
        // End command when we've traveled the specified distance
        .until(
            () -> Math.max(leftEncoder.getDistance(), rightEncoder.getDistance()) >= distanceInches)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> Drive.stopMotor());
  }

  @Override
  public void periodic() {

    double[] botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    // if(Check Bot Pose){
    Pose2d newPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    // resetOdometry(newPose);
    // }

    // resetOdometry(newPose);
    // m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());

    // double leftY = Math.pow(RobotContainer.m_driverController.getLeftY(), 3);
    // double leftX = Math.pow(RobotContainer.m_driverController.getLeftX(), 3);
    // Drive.arcadeDrive(leftX, leftY);
  }
}
