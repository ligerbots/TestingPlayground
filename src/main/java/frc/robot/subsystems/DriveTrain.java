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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
  // These represent our regular encoder objects, which we would
  // create to use on a real robot.
  private Encoder m_leftEncoderSim = new Encoder(0, 1); //4 unique #s to act as ports
  private Encoder m_rightEncoderSim = new Encoder(2, 3);

  // Create our gyro object like we would on a real robot.
  private AnalogGyro m_gyroSim = new AnalogGyro(1); //used to measure tilt

  //make a differential drive
  /*
  This differential drive constructor takes the following parameters:

  The type and number of motors on one side of the drivetrain.

  The gear ratio between the motors and the wheels as output torque over input torque (this number is usually greater than 1 for drivetrains).

  The moment of inertia of the drivetrain (this can be obtained from a CAD model of your drivetrain. Usually, this is between 3 and 8 ).

  The mass of the drivetrain (it is recommended to use the mass of the entire robot itself, as it will more accurately model the acceleration characteristics of your robot for trajectory tracking).

  The radius of the drive wheels.

  The track width (distance between left and right wheels).

  Standard deviations of measurement noise: this represents how much measurement noise you expect from your real sensors. The measurement noise is an array with 7 elements, with each element representing the standard deviation of measurement noise in x, y, heading, left velocity, right velocity, left position, and right position respectively. This option can be omitted in C++ or set to null in Java if measurement noise is not desirable.
  */
  // Create the simulation model of our drivetrain.
  private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
    // Create a linear system from our identification gains.
    LinearSystemId.identifyDrivetrainSystem(Constants.KvLinear, Constants.KaLinear, Constants.KvAngular, Constants.KaAngular),
    DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
    7.29,                    // 7.29:1 gearing reduction.
    0.7112,                  // The track width is 0.7112 meters.
    Units.inchesToMeters(3), // The robot uses 3" radius wheels.
  
    //kvLinear and all of the other stuff are not used anywhere else, just random values for the si
    
      // The standard deviations for measurement noise:
  // x and y:          0.001 m
  // heading:          0.001 rad
  // l and r velocity: 0.1   m/s
  // l and r position: 0.005 m
  VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  //most things not used in real robots

  //create fake motors
  private MotorControllerGroup m_leftMotors = new MotorControllerGroup(new PWMSparkMax(0), new PWMSparkMax(1));
  private MotorControllerGroup m_rightMotors = new MotorControllerGroup(new PWMSparkMax(2), new PWMSparkMax(3));
  //give motors random numbers bc simulate ports


  private DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  private Field2d m_field = new Field2d(); //constructing a field

  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
  //makes odometry, takes in angle reported by gyro
    //Odometry involves using sensors on the robot to create an estimate of the position of the robot on the field.
    //basically is path of robot 


  public DriveTrain() {   //creates a new DriveTrain, initializer

    //m_leftEncoderSim.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    //make a constant for the Math.PI * ...etc...
    m_leftEncoderSim.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
    m_rightEncoderSim.setDistancePerPulse(Constants.ENCODER_DISTANCE_PER_PULSE);
    
    m_leftEncoderSim.reset(); //reset encoders when make this drivetrain, no carry over
    m_rightEncoderSim.reset();

    m_differentialDrive.setSafetyEnabled(false);
    
    SmartDashboard.putData("Field", m_field); //send field over to smartdashboard
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.
    m_odometry.update(m_gyroSim.getRotation2d(),
                      m_leftEncoderSim.getDistance(),
                      m_rightEncoderSim.getDistance());
    //Updates the robot position on the field using distance measurements from encoders.
    m_field.setRobotPose(m_odometry.getPoseMeters());
    //updates what the robot did on the field periodically
  }

  @Override
  public void simulationPeriodic(){
    //  This method will act like periodic but only run in simulations

      // Set the inputs to the system. Note that we need to convert
  // the [-1, 1] PWM signal to voltage by multiplying it by the
  // robot controller voltage.
    m_driveSim.setInputs(m_leftMotors.get() * RobotController.getInputVoltage(),
    m_rightMotors.get() * RobotController.getInputVoltage());
    //set drivetrain to corresponding left and right voltage

  // Advance the model by 20 ms. Note that if you are running this
  // subsystem in a separate thread or have changed the nominal timestep
  // of TimedRobot, this value needs to match it.
    m_driveSim.update(0.02);

  // Update all of our sensors.
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
    
  }

  public void drive(double throttle, double rotate, boolean squaredInput) {
    m_differentialDrive.arcadeDrive(throttle, -rotate, squaredInput);
    //-rotate bc of way a controller works, when pull back the controller, it supposed to move backwards but 
    //actually the rotation is positive bc when controller pulls back it touches to the positive side

    //this is why on sim we swap backwards and forwards
  }

}
