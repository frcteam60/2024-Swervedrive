// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Intake to match paper
// auto starting posistion
// desired positions for sources and amp
// shooter robot lineup
package frc.robot;
// Important !! new Robot!

import java.util.Arrays;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// Network table for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Preferences;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kAutoMiddle = "AutoMiddle";
  private static final String kAutoRight = "AutoRight";
  private static final String kAutoLeft = "AutoLeft";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Preferences pref;

  //private final CANCoder AbsoluteEncoder = new CANCoder(0);
  //private WPI_CANCoder CANCoder = new 

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  float yawOffset;


  // Joystick
  private XboxController controller = new XboxController(0);
  private Joystick joystick = new Joystick(1);
  private Joystick wheelJoystick = new Joystick(2);

  // Shooter and Intake
  ShooterAndIntake shooterAndIntake = new ShooterAndIntake(false, true, false, false, false, 
  0, 1);

  // Climber
  //Climber climber = new Climber(false, false, false, false);

  // Swerve drive
  private WheelDrive backLeft = new WheelDrive(2, 1,0);
  private WheelDrive backRight = new WheelDrive(4, 3,1);
  private WheelDrive frontLeft = new WheelDrive(8, 7,3);
  private WheelDrive frontRight = new WheelDrive(6, 5,2);
  private SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft, gyro.getYaw() + yawOffset);

  // Limelight

  //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  //NetworkTableEntry video = table.getEntry("stream").setNumber(0);
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  NetworkTableEntry robotPoseInTargetSpace = table.getEntry("botpose_targetspace");


  // limelight json
  LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
  double[] botpose2 = LimelightHelpers.getBotPose("limelight");

  //double[] botposeRed = llresults.results.botpose_wpired;  

  //
  double x;
  double y;
  double area;
  double v;
  double hasTarget;
  double ts;
  double botposeRed;
  double botposeBlue;
  double botposeInTargetspace[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,};
  SwerveModulePosition[] swervedrivepositions[];
  boolean blue;




  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("AutoMiddle", kAutoMiddle);
    m_chooser.addOption("AutoRight", kAutoRight);
    m_chooser.addOption("AutoLeft", kAutoLeft);
    SmartDashboard.putData("Auto choices", m_chooser);

    //CameraServer.startAutomaticCapture();
    
    // zeros angle encoders
    frontRight.zeroEncoders(0.675 * 360);
    frontLeft.zeroEncoders(0.37 * 360);
    backRight.zeroEncoders(0.4763 * 360);
    backLeft.zeroEncoders(0.4177 * 360);
    
    // Inverts drive motors
    frontRight.invertDriveMotor(false);
    frontLeft.invertDriveMotor(true);
    backRight.invertDriveMotor(false);
    backLeft.invertDriveMotor(true);
    // Inverts angle motors
    frontRight.invertAngleMotor(false);
    frontLeft.invertAngleMotor(false);
    backRight.invertAngleMotor(false);
    backLeft.invertAngleMotor(false);

    /*
    // zeros angle encoders
    backRight.zeroEncoders(0.3389 * 360);
    backLeft.zeroEncoders(0.7695 * 360);
    frontRight.zeroEncoders(0.0614 * 360);
    frontLeft.zeroEncoders(0.8470 * 360);
    // Inverts drive motors
    frontRight.invertDriveMotor(false);
    frontLeft.invertDriveMotor(true);
    backRight.invertDriveMotor(false);
    backLeft.invertDriveMotor(true);
    // Inverts angle motors
    frontRight.invertAngleMotor(false);
    frontLeft.invertAngleMotor(false);
    backRight.invertAngleMotor(false);
    backLeft.invertAngleMotor(false);
     */

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        //<RED ACTION>
        blue = false;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
      if (ally.get() == Alliance.Blue) {
        //<BLUE ACTION>
        blue = true;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      }
    } 


    
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
    SmartDashboard.putNumber("Yaw", gyro.getYaw() + yawOffset);

    //Drive method
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());

    SmartDashboard.putNumber("forward", swerveDrive.returnForward());
    SmartDashboard.putNumber("strafe", swerveDrive.returnStrafe());
    SmartDashboard.putNumber("turning",swerveDrive.returnTurning());


    //
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    SmartDashboard.putNumber("desiredX", swerveDrive.returnDesiredX());
    SmartDashboard.putNumber("desiredY", swerveDrive.returnDesiredY());
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());
    SmartDashboard.putNumber("currentAngle",backRight.returncurrentAngle());

    // Limelight  
    if (blue){
      botpose2 = LimelightHelpers.getBotPose_wpiBlue("limelight");
      SmartDashboard.putString("botpose2", Arrays.toString(botpose2));

    } else {
      botpose2 = LimelightHelpers.getBotPose_wpiRed("limelight");
      SmartDashboard.putString("botpose2", Arrays.toString(botpose2));
    }


    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    hasTarget = tv.getDouble(0);
    SmartDashboard.putNumber("tv", hasTarget);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);


    botposeInTargetspace = robotPoseInTargetSpace.getDoubleArray(botposeInTargetspace);
    SmartDashboard.putNumber("robot from target yaw", botposeInTargetspace[5]);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("Shooter Angle encoder", shooterAndIntake.returnAngle());

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
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        //<RED ACTION>
        blue = false;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
      if (ally.get() == Alliance.Blue) {
        //<BLUE ACTION>
        blue = true;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      }
    } 

    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case kAutoMiddle:
        // In the middle of the subwoofer
        if (blue){
          yawOffset = 0;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, new Pose2d(1.365, 5.548, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        } else {
          yawOffset = 0;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, null, new Pose2d(1.365, 2.656, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        }
        break;
      case kAutoRight:
        // auto right
        // In the right of the subwoofer
        if (blue){
          yawOffset = -60;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, null, new Pose2d(0.683, 4.365, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        } else {
          yawOffset = -60;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, null, new Pose2d(0.683, 1.474, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        }
        break;
      case kAutoLeft:
        // auto left
        // On the left of the subwoofer
        if (blue){
          yawOffset = 60;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, null, new Pose2d(0.683, 6.731, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        } else {
          yawOffset = 60;
          swerveDrive.setPosition(gyro.getYaw() + yawOffset, null, new Pose2d(0.683, 3.839, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));
        }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        yawOffset = 0;
        break;
    }

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kAutoMiddle:
        // Put custom auto code here
        if (blue == true){

        } else {

        }
        break;
      case kAutoRight:
        // auto right
        if (blue == true){

        } else {
          
        }
        break;
      case kAutoLeft:
        // auto left
        if (blue == true){

        } else {
          
        }
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
    swerveDrive.setDesiredYaw(gyro.getYaw() + yawOffset);

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        //<RED ACTION>
        blue = false;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
      if (ally.get() == Alliance.Blue) {
        //<BLUE ACTION>
        blue = true;
        //NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      }
    } 

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // updates odemetery
    SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    swerveDrive.resetPosition((gyro.getYaw() + yawOffset), modulePositions, new Pose2d(botposeInTargetspace[0], botposeInTargetspace[1], new Rotation2d(-(botposeInTargetspace[5]/360 * 2 * Math.PI))), new Pose2d(botpose2[0], botpose2[1], new Rotation2d(-(botpose2[5]/360 * 2 * Math.PI))), hasTarget); 
    
    
    if (controller.getXButton()){
      // if override pressed
      swerveDrive.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), gyro.getYaw() + yawOffset);

    //} else if (controller.getBButton()){
    
      // line up robot angle for shooter
      /*swerveDrive.setDesiredYaw(0);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);*/
      // write variable linedUp
    } else if(joystick.getRawButtonPressed(5)){
      // Go to far source
      if (blue){
        swerveDrive.setDesiredPosistion(15.96, 1.274, 120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      } else {
        swerveDrive.setDesiredPosistion(15.96, 6.93, -120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      }
    } else if(joystick.getRawButtonPressed(3)){
      // Go to close source
      if (blue){
        swerveDrive.setDesiredPosistion(14.854, 0.636, 120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      } else {
        swerveDrive.setDesiredPosistion(14.854, 4.568, -120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      }
    } else if(joystick.getRawButtonPressed(4)){
      // Go to subwoofer
      if (blue){
        swerveDrive.setDesiredPosistion(1.365, 5.48, 0);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      } else {
        swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      }
    } else if(joystick.getRawButtonPressed(2)){
      // Go to amp
      if (blue){
        swerveDrive.setDesiredPosistion(1.842, 7.53, 90);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      } else {
        swerveDrive.setDesiredPosistion(1.842, 0.451, -90);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw());
      }
    } else if (joystick.getPOV() == 0){
      //North
      swerveDrive.setDesiredYaw(0);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);
    } else if(joystick.getPOV() == 180){
      //South
      swerveDrive.setDesiredYaw(180);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);
    } else if(joystick.getPOV() == 90){
      //East
      swerveDrive.setDesiredYaw(90);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);
    } else if(joystick.getPOV() == 270){
      //West
      swerveDrive.setDesiredYaw(-90);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);
    } else {
      // primary driver inputs
      swerveDrive.drive(joystick.getX(), joystick.getY(), wheelJoystick.getRawAxis(0), gyro.getYaw() + yawOffset);
    }
    

    // Shooter angle
    if (joystick.getRawButton(5) || joystick.getRawButton(3)){
      // Source pick up
      shooterAndIntake.setAngle(20);
      shooterAndIntake.shooter(-0.5);
    } else if(controller.getBButton()){
      // line up shooter for speaker
      shooterAndIntake.setAngle(31);
      //shooterAndIntake.shootInSpeaker(true, botposeInTargetspace);
    } else if(controller.getAButton()){
      // Amp
      shooterAndIntake.setAngle(96);
    } else if (Math.abs(controller.getLeftY()) > 0.02){
      shooterAndIntake.angle(controller.getLeftY());
    } else {
      shooterAndIntake.angle(0);
    }

    // Shooter
    if (joystick.getRawButton(5) || joystick.getRawButton(3)){
      // Source
    } else if (controller.getYButton()){
      // Low power or variable shooter
      // 0.15 good low power
      shooterAndIntake.shooter(Preferences.getDouble("ShooterPower", 0.7));
    } else if(controller.getBButton()){
      // Speaker subwoofer
      shooterAndIntake.shooter(0.33);
    } else if (controller.getAButton()){
      // amp shoot
      shooterAndIntake.shooter(15);
      // If shooter at angle shoot in amp
      //shooterAndIntake.shootInAmp();
    } else if (controller.getRightTriggerAxis() == 0.5){
      // Shoot out
      shooterAndIntake.shooter(controller.getRightTriggerAxis());
    } else if(controller.getRightBumper()){
      // Reverse shooter
      shooterAndIntake.shooter(-0.5);
    } else {
      shooterAndIntake.shooter(0);
    }
    

    /*(if ( controller.getRightTriggerAxis() >= 0.5){
      shooterAndIntake.shooter(controller.getRightTriggerAxis());
    } else if(controller.getRightBumper()){
      shooterAndIntake.shooter(-0.5);
    } else {
      shooterAndIntake.shooter(0);
    }*/
    
    // Intake
    if (controller.getLeftTriggerAxis() >= 0.5){
      shooterAndIntake.intake(1);
      //swerveDrive.setTurnPoint(new Pose2d(0, 0, null)); 
    } else if(controller.getLeftBumper()){
      shooterAndIntake.intakeSpeed(Preferences.getDouble("IntakeTop", 0.7), Preferences.getDouble("IntakeBottom", 0.7));
    }else {
      shooterAndIntake.intake(0);
    }

    /*// intake
    if (controller.getLeftTriggerAxis() == 1){
      // Intake
      shooterAndIntake.intake(1);
    } else if (controller.getLeftBumper()){
      // Reverse intake
      shooterAndIntake.intake(-1);
    }*/

    // climber
    /*if (controller.getPOV() == 0){
      climber.climb(1);
    } else if (controller.getPOV() == 180) {
      climber.climb(-1);
    }*/

    if (wheelJoystick.getRawButtonPressed(1)){
      //left bumper
      gyro.reset();
    }

    if (wheelJoystick.getRawButtonPressed(0)){
      // right bumper
      frontRight.resetInvert(false, false);
      frontLeft.resetInvert(true, false);
      backRight.resetInvert(false, false);
      backLeft.resetInvert(true, false);
    }
    
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
