// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.SPI; // required for ADIS IMUs
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.ADIS16448_IMU; 
import edu.wpi.first.wpilibj.ADIS16470_IMU;  
import org.photonvision.PhotonCamera; //https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json

public class Robot extends TimedRobot {
  private final Spark m_rightDrive = new Spark(0);
  private final Spark m_leftDrive = new Spark(1);
  //private final Victor shooter = new Victor(3);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_Xbox = new XboxController(0);
  private final Joystick m_Jstick = new Joystick(1);
  
  private final Timer m_timer = new Timer();
  private final Encoder right_encoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);
  private final Encoder left_encoder = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
  //public static final ADIS16448_IMU imu = new ADIS16448_IMU(ADIS16448_IMU.IMUAxis.kX, SPI.Port.kMXP, ADIS16448_IMU.CalibrationTime._1s);
  public static final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kX,ADIS16470_IMU.IMUAxis.kY,ADIS16470_IMU.IMUAxis.kZ, SPI.Port.kOnboardCS0, ADIS16470_IMU.CalibrationTime._1s);
  PhotonCamera camera = new PhotonCamera("Camera_Module_v1");    //wide angle on rPi3 is called "OV5647"
                                                                            //other camera on rPi4 is called "Camera_Module_v3"
  public double forwardSpeed,ccYawEffort,ccYaw;
  public static double pTargetYaw;
  public static boolean pHasTarget;
  public static double Yaw_P = 0.014;
  public static double Yaw_I = 0.01;  
  public static double Yaw_D = 0.0015;
  public static double Yaw_Bias = 0.20;
  public static double adjustable = 0.02;
    
  PIDController turnController = new PIDController(Yaw_P, Yaw_I, Yaw_D);
  // pose estimator
  

  @Override
  public void robotInit() {
    imu.calibrate();
    m_rightDrive.setInverted(true);
    right_encoder.setSamplesToAverage(5);
    left_encoder.setSamplesToAverage(5);
    right_encoder.setDistancePerPulse(10.0 * 0.3048 /13280); // the pulses where averaged from l&r ca 13280 of 10' (4 x 2' tiles)
    left_encoder.setDistancePerPulse( 10.0 * 0.3048 /13280); // the factor 0.3048 converts feet into metres
    right_encoder.setMinRate(1.0);
    left_encoder.setMinRate(1.0);
    right_encoder.reset();
    left_encoder.reset();

    
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(imu); 
    SmartDashboard.putData(right_encoder);
    SmartDashboard.putData(left_encoder);
    SmartDashboard.putData(turnController);
  }  
  
  @Override
  public void robotPeriodic(){
    var result = camera.getLatestResult();
    pHasTarget = result.hasTargets();
    if (pHasTarget) {
      pTargetYaw = result.getBestTarget().getYaw();
      SmartDashboard.putNumber("TargetID", result.getBestTarget().getFiducialId());    
    }
    ccYaw = imu.getAngle(ADIS16470_IMU.IMUAxis.kY) % 360;
    ccYaw =  (ccYaw > 180) ? ccYaw-360:ccYaw;
    ccYaw = (ccYaw < -180) ? ccYaw+360:ccYaw;
    
    
    
    
    SmartDashboard.putBoolean("Has Target",pHasTarget);
    
    SmartDashboard.putNumber("Target Yaw", pTargetYaw);    
    
    SmartDashboard.putNumber("Angle", -imu.getAngle(ADIS16470_IMU.IMUAxis.kY));
    SmartDashboard.putNumber("Yaw", ccYaw);
    //SmartDashboard.putBoolean("Button 1",m_Jstick.getRawButton(1));
    //SmartDashboard.putNumber("Joy Throttle",m_Jstick.getRawAxis(2));
    adjustable=1-((m_Jstick.getRawAxis(2)+1.0)*0.5);
    SmartDashboard.putNumber("adjustable", adjustable);
    
  }
  @Override
  public void autonomousInit() {
    m_timer.reset();
    right_encoder.reset();
    left_encoder.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   if (right_encoder.getDistance() < (1.00)) {       //only  drive when the encoder reads less than 1.00 m -- NEEDS ENCODERS TO PREVENT WILD ROBOT --
  //  if (m_timer.get() < 2) {         //only  drive when the timer is less than 2 seconds
      m_robotDrive.arcadeDrive(0.5, 0.0,false); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  @Override
  public void teleopPeriodic() {
  
    if (m_Xbox.getLeftBumper()) {
        right_encoder.reset();
        left_encoder.reset();
    }
    if (m_Xbox.getRightBumper()) {
      imu.reset();
  }
  forwardSpeed = m_Xbox.getRightTriggerAxis()-m_Xbox.getLeftTriggerAxis();
  ccYawEffort = -m_Xbox.getRightX()*0.3;

  if (m_Xbox.getYButton()) {
  ccYawEffort = (Math.abs(ccYaw) < 10)
   ? turnController.calculate(ccYaw)+Math.copySign(Yaw_Bias,turnController.calculate(ccYaw))
   : -.4*Math.signum(ccYaw) ;
  }
  if (m_Xbox.getYButtonReleased()) {turnController.reset();}

  m_robotDrive.arcadeDrive(forwardSpeed, ccYawEffort,false); 
  }
}
