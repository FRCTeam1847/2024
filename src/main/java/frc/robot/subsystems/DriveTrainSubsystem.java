// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class DriveTrainSubsystem extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final CANSparkMax m_leftDrive1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive1 = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax m_leftDrive2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax m_rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);

  private final DifferentialDrive Drive = new DifferentialDrive(m_leftDrive1, m_rightDrive1);

  private final Encoder leftEncoder = new Encoder(2, 3);
  private final Encoder rightEncoder = new Encoder(1, 0);

  private static final double cpr = 360;
  private static final double whd = 6;

  // private DifferentialDriveKinematics m_Kinematics = new DifferentialDriveKinematics(32.375);

  WPI_PigeonIMU _pidgety = new WPI_PigeonIMU(0);

  // private DifferentialDrivePoseEstimator m_DifferentialDrivePoseEstimator;

  private Field2d m_field = new Field2d();
  private Pose2d pose = new Pose2d();

  double[] botPose = table.getEntry("botPose").getDoubleArray(new double[6]);

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    leftEncoder.setDistancePerPulse(Math.PI * whd / cpr);
    rightEncoder.setDistancePerPulse(Math.PI * whd / cpr);
    rightEncoder.setReverseDirection(true);
    leftEncoder.setReverseDirection(true);

    m_leftDrive2.follow(m_leftDrive1);
    m_rightDrive2.follow(m_rightDrive1);

    // m_DifferentialDrivePoseEstimator
    // = new DifferentialDrivePoseEstimator(m_Kinematics, _pidgety.getRotation2d(),
    // leftEncoder.getDistance(), rightEncoder.getDistance());

  }

  public void ArcadeDrive(double X, double Z) {
    Drive.arcadeDrive(X, Z);
  }

  @Override
  public void periodic() {
    botPose = table.getEntry("botpose").getDoubleArray(new double[6]);

    Pose2d robotPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[2]));
    m_field.setRobotPose(robotPose);

    // m_DifferentialDrivePoseEstimator.update(_pidgety.getRotation2d(), leftEncoder.getDistance(),
    //     rightEncoder.getDistance());
    SmartDashboard.putData("Field", m_field);

    // m_DifferentialDrivePoseEstimator.addVisionMeasurement(null, cpr, null);
    double leftX = Math.pow(RobotContainer.m_driverController.getLeftX(), 3);
    double leftY = Math.pow(RobotContainer.m_driverController.getLeftY(), 3);
    Drive.arcadeDrive(leftX, leftY);
  }
}
