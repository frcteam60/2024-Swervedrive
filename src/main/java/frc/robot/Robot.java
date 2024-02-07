// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// set spark max encoder from absolute encoder
// add motor CAN numbers
// add motor watch dogs
// configAbsoluteSensorRange
package frc.robot;

import java.util.Arrays;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI;

// Network table for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.DoubleUnaryOperator;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.Quaternion;

import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.
//import com.ctre.phoenix.sensors.
/*import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;*/

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //private final CANCoder AbsoluteEncoder = new CANCoder(0);
  //private WPI_CANCoder CANCoder = new 

  AHRS gyro = new AHRS(SPI.Port.kMXP);


  // Joystick
  private XboxController controller = new XboxController(0);

  // Swerve drive
  private WheelDrive backLeft = new WheelDrive(2, 1,0);
  private WheelDrive backRight = new WheelDrive(4, 3,1);
  private WheelDrive frontLeft = new WheelDrive(8, 7,3);
  private WheelDrive frontRight = new WheelDrive(6, 5,2);
  private SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft, gyro.getYaw());

  // Limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledMode = table.getEntry("ledMode");

  // limelight json
  LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
  double[] botpose2 = LimelightHelpers.getBotPose("limelight");

  //double[] botposeRed = llresults.results.botpose_wpired;  

  //
  double x;
  double y;
  double area;
  double v;
  double ts;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //AbsoluteEncoder.configSensorDirection(false);
    //AbsoluteEncoder.configAbsoluteSensorRange
    //AbsoluteEncoder.getAbsolutePosition();
    
    backRight.zeroEncoders(0.3389 * 360);
    backLeft.zeroEncoders(0.7695 * 360);
    frontRight.zeroEncoders(0.0614 * 360);
    frontLeft.zeroEncoders(0.8470 * 360);

    
    frontRight.invertDriveMotor(false);
    frontLeft.invertDriveMotor(true);
    backRight.invertDriveMotor(false);
    backLeft.invertDriveMotor(true);

    frontRight.invertAngleMotor(false);
    frontLeft.invertAngleMotor(false);
    backRight.invertAngleMotor(false);
    backLeft.invertAngleMotor(false);

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *s
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Relative Encoders
    SmartDashboard.putNumber("backLeft relative encoder", backLeft.returnRelative());
    SmartDashboard.putNumber("backRight relative encoder", backRight.returnRelative());
    SmartDashboard.putNumber("frontRight relative encoder", frontRight.returnRelative());
    SmartDashboard.putNumber("frontLeft relative encoder", frontLeft.returnRelative());
    // Absolute Encoders
    SmartDashboard.putNumber("backLeft AbsoluteEncoder", backLeft.returnabsolute());
    SmartDashboard.putNumber("backRight AbsoluteEncoder", backRight.returnabsolute());
    SmartDashboard.putNumber("frontLeft AbsoluteEncoder", frontLeft.returnabsolute());
    SmartDashboard.putNumber("frontRight AbsoluteEncoder", frontRight.returnabsolute());

    // Gyro angle
    SmartDashboard.putNumber("Yaw", gyro.getYaw());

    //Drive method
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());

    SmartDashboard.putNumber("forward", swerveDrive.returnForward());
    SmartDashboard.putNumber("strafe", swerveDrive.returnStrafe());
    SmartDashboard.putNumber("turning",swerveDrive.returnTurning());


    //
    SmartDashboard.putNumber("desiredX", swerveDrive.returnDesiredX());
    SmartDashboard.putNumber("desiredY", swerveDrive.returnDesiredY());
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());
    SmartDashboard.putNumber("currentAngle",backRight.returncurrentAngle());

    // Limelight  
    double[] botpose2 = LimelightHelpers.getBotPose("limelight");
    SmartDashboard.putString("botpose2", Arrays.toString(botpose2));
    

    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    /*
    SmartDashboard.putString("rotation2d", swerveDrive.returnRotation().toString());
    SmartDashboard.putNumber("robot X", swerveDrive.returnX());
    SmartDashboard.putNumber("robot Y", swerveDrive.returnY());*/

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    swerveDrive.setDesiredYaw(gyro.getYaw());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (controller.getLeftBumper()){
      swerveDrive.setDesiredPosistion(0, 0, 0);
      swerveDrive.driveToPosistion(0, 0, 0, gyro.getYaw());
    } else {

      if (controller.getYButton()){
        //North
        swerveDrive.setDesiredYaw(0);
      } else if(controller.getAButton()){
        //South
        swerveDrive.setDesiredYaw(180);
      }else if(controller.getBButton()){
        //East
        swerveDrive.setDesiredYaw(90);
      } else if(controller.getXButton()){
        //West
        swerveDrive.setDesiredYaw(-90);
      } else {
        swerveDrive.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), gyro.getYaw());
      }
    }
    
    

    /*if (controller.getRightBumper()){
      gyro.reset();
    }*/

    /*
    frontRight.resetInvert(false, false);
    frontLeft.resetInvert(true, false);
    backRight.resetInvert(false, false);
    backLeft.resetInvert(true, false);
    */

    SmartDashboard.putString("rotation2d", swerveDrive.returnRotation().toString());
    SmartDashboard.putString("field? rotation2d", swerveDrive.returnfieldPose2d().toString());
    SmartDashboard.putNumber("robot X", swerveDrive.returnX());
    SmartDashboard.putNumber("robot Y", swerveDrive.returnY());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
