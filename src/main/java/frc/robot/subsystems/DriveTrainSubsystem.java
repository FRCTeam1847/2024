// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private final CANSparkMax m_leftDrive1;
  private final CANSparkMax m_rightDrive1;
  private final CANSparkMax m_leftDrive2;
  private final CANSparkMax m_rightDrive2;

  public final Encoder leftEncoder;
  public final Encoder rightEncoder;

  private static final double cpr = 360;
  private static final double whd = 6;
  private static final double distancePerPulse = Math.PI * (whd / cpr);

  private final DifferentialDrive Drive;
  public final WPI_PigeonIMU gyro;

  private Field2d m_field = new Field2d();

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    // Set devices
    m_leftDrive1 = new CANSparkMax(1, MotorType.kBrushless);
    m_rightDrive1 = new CANSparkMax(4, MotorType.kBrushless);
    m_leftDrive2 = new CANSparkMax(2, MotorType.kBrushless);
    m_rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);
    m_leftDrive1.setIdleMode(IdleMode.kBrake);
    m_leftDrive2.setIdleMode(IdleMode.kBrake);
    m_rightDrive1.setIdleMode(IdleMode.kBrake);
    m_rightDrive2.setIdleMode(IdleMode.kBrake);

    leftEncoder = new Encoder(2, 3);
    rightEncoder = new Encoder(1, 0);
    gyro = new WPI_PigeonIMU(0);
    gyro.reset();
    // Set up Drive
    m_rightDrive1.setInverted(true);
    m_leftDrive2.follow(m_leftDrive1);
    m_rightDrive2.follow(m_rightDrive1);
    Drive = new DifferentialDrive(m_leftDrive1, m_rightDrive1);
    // set up encoders
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setReverseDirection(true);
    leftEncoder.setReverseDirection(true);
    leftEncoder.reset();
    rightEncoder.reset();
    // Set up field on dashboard
    SmartDashboard.putData("Field", m_field);

  }

  public static double limitedCube(double x, double speed) {
    // Calculate x^3
    double result = Math.pow(x, 3);
    // Check if result is below the minimum limit
    if (result < -speed) {
      return -speed;
    }
    // Check if result is above the maximum limit
    else if (result > speed) {
      return speed;
    }
    // Return the result if it's within the limits
    return result;
  }
  public void ArcadeDrive(double X, double Z, boolean square) {
    Drive.arcadeDrive(X, Z, square);
  }

  public void ArcadeDrive(double X, double Z) {
    Drive.arcadeDrive(X, Z);
  }

  public void Stop(){
    Drive.stopMotor();
  }

  // public Command getPosition(){}

  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // Set arcade drive with a cubic function
    return run(() -> {
      double xSpeed = limitedCube(fwd.getAsDouble(), 1);
      double zRotation = limitedCube(rot.getAsDouble(), 1);
      Drive.arcadeDrive(xSpeed, zRotation);
    })
        .withName("arcadeDrive");
  }

  @Override
  public void periodic() {
    double[] botPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    Pose2d newPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    m_field.setRobotPose(newPose);

    SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
    SmartDashboard.putNumber("Gyro", gyro.getAngle());
    SmartDashboard.putNumber("Camera TX", table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("Camera Ty", table.getEntry("ty").getDouble(0.0));
  }
}
