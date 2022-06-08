// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
  
  private Encoder m_leftEncoder = new Encoder(0, 1);
  private Encoder m_rightEncoder = new Encoder(2, 3);

  private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
  private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

  private AnalogGyro m_gyro = new AnalogGyro(1);

  private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    LinearSystemId.identifyDrivetrainSystem(Constants.KvLinear, Constants.KvLinear, Constants.KvAngular, Constants.KaAngular),
    DCMotor.getNEO(2),
    7.29,
    0.7112,
    Units.inchesToMeters(3),

    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
     
  private PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private Field2d m_field = new Field2d();

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

  public DriveTrain() {
    m_leftEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
    m_rightEncoder.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic(){
    m_driveSim.setInputs(m_leftMotor.get()* RobotController.getInputVoltage(), m_rightMotor.get()* RobotController.getInputVoltage());

    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

  }
}
