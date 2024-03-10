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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
// Network table for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private Timer timer = new Timer();

  Preferences pref;

  public int autoStep = 1;
  public boolean stepOne = false;
  public boolean stepTwo = false;
  public boolean stepThree = false;
  public boolean stepFour = false;
  public boolean stepFive = false;
  public boolean stepSix = false;
  public boolean stepSeven = false;
  public boolean stepEight = false;

  //private final CANCoder AbsoluteEncoder = new CANCoder(0);
  //private WPI_CANCoder CANCoder = new 

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  float yawOffset;

  ColorSensorV3 colorSensor = new ColorSensorV3(Port.kMXP);

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

  double autoStartingPositionX;
  double autoSecondPositionX;
  double autoThirdPositionX;
  double autoStartingPositionY;
  double autoSecondPositionY;
  double autoThirdPositionY;
  double autoStartingPositionRotation;
  double autoSecondPositionRotation;
  double autoThirdPositionRotation;
  
  double autoShootingThirdPositionX;
  double autoShootingThirdPositionY;
  double autoShootingThirdPositionRotation;

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
    shooterAndIntake.setAngleEncoder(21.8);
    
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
    SmartDashboard.putNumber("X error", swerveDrive.returnXError());
    SmartDashboard.putNumber(" eYrror", swerveDrive.returnYError());
    SwerveModulePosition[] modulePositions = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    swerveDrive.resetPosition((gyro.getYaw() + yawOffset), modulePositions, new Pose2d(botposeInTargetspace[0], botposeInTargetspace[1], new Rotation2d(-(botposeInTargetspace[5]/360 * 2 * Math.PI))), new Pose2d(botpose2[0], botpose2[1], new Rotation2d(-(botpose2[5]/360 * 2 * Math.PI))), hasTarget); 
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

    // Color sensor
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    SmartDashboard.putNumber("IR", colorSensor.getIR());

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

    SmartDashboard.putString("rotation2d", swerveDrive.returnRotation().toString());
    SmartDashboard.putNumber("robot X", swerveDrive.returnX());
    SmartDashboard.putNumber("robot Y", swerveDrive.returnY());

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
    timer.stop();
    timer.start();
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
    // Sets positions on the field based on our auto selected and alliance color
    switch (m_autoSelected){
      case kAutoMiddle:
        if (blue){
        autoStartingPositionX = 1.365;
        autoStartingPositionY = 5.548;
        autoStartingPositionRotation = 0;
        // Middle position doesn't have a second position
        
        autoThirdPositionX = 2.623;
        autoThirdPositionY = 5.548;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 5.548;
        autoShootingThirdPositionRotation = 0;
        } else {
        autoStartingPositionX = 1.365;
        autoStartingPositionY = 2.656;
        autoStartingPositionRotation = 0;
        // Middle position doesn't have a second position
        
        autoThirdPositionX = 2.623;
        autoThirdPositionY = 2.656;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 2.656;
        autoShootingThirdPositionRotation = 0;
        }
      break;
      case kAutoRight:
        if (blue){
        autoStartingPositionX = 0.683;
        autoStartingPositionY = 4.365;
        autoStartingPositionRotation = -60;
        
        autoSecondPositionX = 0.836;
        autoSecondPositionY = 4.1;
        autoSecondPositionRotation = 0;

        autoThirdPositionX = 2.623;
        autoThirdPositionY = 4.1;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 4.1;
        autoShootingThirdPositionRotation = -31;
        } else {
        autoStartingPositionX = 0.683;
        autoStartingPositionY = 1.474;
        autoStartingPositionRotation = -60;
       
        autoSecondPositionX = 0.836;
        autoSecondPositionY = 1.209;
        autoSecondPositionRotation = 0;

        autoThirdPositionX = 2.623;
        autoThirdPositionY = 1.209;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 1.209;
        autoShootingThirdPositionRotation = -31;
        }
      break;
      case kAutoLeft:
      if (blue){
        autoStartingPositionX = 0.683;
        autoStartingPositionY = 6.731;
        autoStartingPositionRotation = 60;
        
        autoSecondPositionX = 0.836;
        autoSecondPositionY = 6.996;
        autoSecondPositionRotation = 0;

        autoThirdPositionX = 2.623;
        autoThirdPositionY = 6.996;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 6.996;
        autoShootingThirdPositionRotation = 31;
        } else {
        autoStartingPositionX = 0.683;
        autoStartingPositionY = 3.839;
        autoStartingPositionRotation = 60;
       
        autoSecondPositionX = 0.836;
        autoSecondPositionY = 4.104;
        autoSecondPositionRotation = 0;

        autoThirdPositionX = 2.623;
        autoThirdPositionY = 4.104;        
        autoThirdPositionRotation = 0;
        
        autoShootingThirdPositionX = 2.623;
        autoShootingThirdPositionY = 4.104;
        autoShootingThirdPositionRotation = 31;
        }
      break;
    }
    yawOffset = (float) autoStartingPositionRotation;
    swerveDrive.setPosition(gyro.getYaw() + yawOffset, new SwerveModulePosition[]{frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()}, 
    new Pose2d(autoStartingPositionX, autoStartingPositionY, new Rotation2d(-(gyro.getYaw() + yawOffset)* 2 * Math.PI)));

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    SmartDashboard.putString("Auto selected", m_autoSelected);

    switch (m_autoSelected) {
      case kAutoMiddle:
        switch (autoStep) {
          case 1:
          // Step one
          shooterAndIntake.setAngle(57);
          shooterAndIntake.shooterToSpeed(colorSensor.getIR(), 0.33);
          // Step 1 check
          if (Math.abs(shooterAndIntake.returnAngle() - 57) <= 2){
          //if (timer.get() >= 3){
            // if shooter angle is close to 3
            System.out.println("Start of step 2");
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 2;
          }
          break;
          case 2:
          //Step two
          // shoot
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.33);
          // Step 2 check
          if (timer.get() >= 2){
            System.out.println("Start of step 3");
            // if 2 or more seconds on step 2
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 3;
          }
          break;
          case 3:
          // Step three
          // middle doesn't have 2nd position
          if (blue == true){
            // Blue
          } else {
            // Red
          }
          // Step 3 check
          if (true){
            System.out.println("Start of step 4");
            // if at 2nd position
            autoStep = 4;
          }
          break;
          case 4:
          // Step four
          // 3rd position and intake
          shooterAndIntake.setAngle(25);
          shooterAndIntake.intake(1);
          if (blue == true){
            System.out.println("blue");
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 5.548, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((5.548 - swerveDrive.returnY())*(5.548 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              System.out.println("Start of step 5 second shot");
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          } else {
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 2.56, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((2.56 - swerveDrive.returnY())*(2.56 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              System.out.println("Start of step 5 second shot");
              swerveDrive.drive(0, 0, 0, gyro.getYaw() + yawOffset);
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          }
          break;
          /*
          case 5:
            shooterAndIntake.intake(-1);
            shooterAndIntake.setAngle(37);
            if (timer.get() >= 0.5){
              System.out.println("Start of step 6");
              shooterAndIntake.intake(0);
              autoStep = 6;
            }
          break;
          case 6:
          // Step six
          // line up to shoot
          // power up shooter
          shooterAndIntake.shooter(0.7);
          // set shooter angle
          shooterAndIntake.setAngle(37);
          /* Back to subwoofer
          if (blue){
            swerveDrive.setDesiredPosistion(1.365, 5.548, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
            swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }*//*
          // Step 6 check
          if (Math.abs(shooterAndIntake.returnAngle() - 37) <= 2){
            // if lined up
            System.out.println("Start of step 7");
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 7;
          } 
          break;
          case 7:
          // Step seven
          // shoot second note
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.7);
          // Step 7 check
          if (timer.get() >= 2){
            System.out.println("Start of step 8");
            shooterAndIntake.angle(0);
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 8;
          }
          break;
          case 8:
          // Step eight
          if (blue == true){
            //swerveDrive.setDesiredPosistion(3.623, 5.548, 0);
            //swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
            //swerveDrive.setDesiredPosistion(3.623, 2.56, 0);
            //swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }
          break;*/
        }

      case kAutoRight:
        // auto right
        switch (autoStep) {
          case 1:
          // Step one
          // auto right
          shooterAndIntake.setAngle(57);
          shooterAndIntake.shooterToSpeed(colorSensor.getIR(), 0.33);
          // Step 1 check
          if (Math.abs(shooterAndIntake.returnAngle() - 57) <= 2){
            // if shooter angle is close to 31
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 2;
          }
          break;
          case 2:
          // Step two
          // auto right
          // shoot
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.33);
          // Step 2 check
          if (timer.get() >= 2){
            // if 2 or more seconds on step 2
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 3;
          }
          break;
          case 3:
          // Step three
          // auto right
          // second position
          shooterAndIntake.setAngle(25);
          if (blue == true){
            // drive to second position
            swerveDrive.setDesiredPosistion(0.836, 4.1, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 3 check
            if (Math.sqrt(((0.836 - swerveDrive.returnX())*(0.836 - swerveDrive.returnX())) + ((4.1 - swerveDrive.returnY())*(4.1 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              autoStep = 4;
            } 
          } else {
            // drive to second position
            swerveDrive.setDesiredPosistion(0.836, 1.209, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 3 check
            if (Math.sqrt(((0.836 - swerveDrive.returnX())*(0.836 - swerveDrive.returnX())) + ((1.209 - swerveDrive.returnY())*(1.209 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              autoStep = 4;
            } 
          }
     
          break;
          case 4:
          // Step four
          // auto right
          // 3rd position and intake
          shooterAndIntake.setAngle(25);
          shooterAndIntake.intake(1);
          if (blue == true){
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 4.1, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((4.1 - swerveDrive.returnY())*(4.1 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          } else {
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 2.656, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((2.656 - swerveDrive.returnY())*(2.656 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          }
          break;
          case 5:
            // Step five
            // auto right
            // reverse intake
            shooterAndIntake.intake(-1);
            shooterAndIntake.setAngle(37);
            if (blue){
              swerveDrive.setDesiredPosistion(2.623, 4.1, -31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            } else {
              swerveDrive.setDesiredPosistion(2.623, 2.656, -31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            }
            // Step five check
            if (timer.get() >= 0.5){
              shooterAndIntake.intake(0);
              autoStep = 6;
            }
          break;
          case 6:
          // Step six
          // auto right
          // line up to shoot
          // power up shooter
          shooterAndIntake.shooter(0.7);
          // set shooter angle
          shooterAndIntake.setAngle(33);
          if (blue){
              swerveDrive.setDesiredPosistion(2.623, 4.1, -31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            } else {
              swerveDrive.setDesiredPosistion(2.623, 2.656, -31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            }
          /* Back to subwoofer
          if (blue){
            swerveDrive.setDesiredPosistion(1.365, 5.548, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
            swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }*/
          // Step 6 check
          if (Math.abs(shooterAndIntake.returnAngle() - 33) <= 2){
            // if lined up
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 7;
          } 
          break;
          case 7:
          // Step seven
          // auto right
          // shoot second note
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.7);
          // Step 7 check
          if (timer.get() >= 2){
            shooterAndIntake.angle(0);
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 8;
          }
          break;
          case 8:
          // Step eight
          // auto right
          if (blue == true){
            swerveDrive.setDesiredPosistion(3.623, 4.1, -31);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
            swerveDrive.setDesiredPosistion(3.623, 2.656, -31);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }
          break;
        }
       
        break;
      case kAutoLeft:
        // auto left
        switch (autoStep) {
          case 1:
          // Step one
          // auto left
          shooterAndIntake.setAngle(57);
          shooterAndIntake.shooterToSpeed(colorSensor.getIR(), 0.33);
          // Step 1 check
          if (Math.abs(shooterAndIntake.returnAngle() - 57) <= 2){
            // if shooter angle is close to 31
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 2;
          }
          break;
          case 2:
          // Step two
          // auto left
          // shoot
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.33);
          // Step 2 check
          if (timer.get() >= 2){
            // if 2 or more seconds on step 2
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 3;
          }
          break;
          case 3:
          // Step three
          // auto left
          // second position
          shooterAndIntake.setAngle(25);
          if (blue == true){
            // drive to second position
            swerveDrive.setDesiredPosistion(0.836, 6.996, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 3 check
            if (Math.sqrt(((0.836 - swerveDrive.returnX())*(0.836 - swerveDrive.returnX())) + ((6.996 - swerveDrive.returnY())*(6.996 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              autoStep = 4;
            } 
          } else {
            // drive to second position
            swerveDrive.setDesiredPosistion(0.836, 4.104, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 3 check
            if (Math.sqrt(((0.836 - swerveDrive.returnX())*(0.836 - swerveDrive.returnX())) + ((4.104 - swerveDrive.returnY())*(4.104 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              autoStep = 4;
            } 
          }
     
          break;
          case 4:
          // Step four
          // auto left
          // 3rd position and intake
          shooterAndIntake.setAngle(25);
          shooterAndIntake.intake(1);
          if (blue == true){
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 6.996, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((6.996 - swerveDrive.returnY())*(6.996 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          } else {
            // drive to note
            swerveDrive.setDesiredPosistion(2.623, 4.104, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            // Step 4 check
            if (Math.sqrt(((2.623 - swerveDrive.returnX())*(2.623 - swerveDrive.returnX())) + ((4.104 - swerveDrive.returnY())*(4.104 - swerveDrive.returnY()))) <= 0.05){
              // if at 3rd position
              shooterAndIntake.intake(0);
              timer.reset();
              autoStep = 5;
            } 
          }
          break;
          case 5:
            // Step five
            // auto left
            // reverse intake
            shooterAndIntake.intake(-1);
            shooterAndIntake.setAngle(37);
            if (blue){
              swerveDrive.setDesiredPosistion(2.623, 6.996, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            } else {
              swerveDrive.setDesiredPosistion(2.623, 4.104, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            }
            // Step five check
            if (timer.get() >= 0.5){
              shooterAndIntake.intake(0);
              autoStep = 6;
            }
          break;
          case 6:
          // Step six
          // auto left
          // line up to shoot
          // power up shooter
          shooterAndIntake.shooter(0.7);
          // set shooter angle
          shooterAndIntake.setAngle(33);
          if (blue){
              swerveDrive.setDesiredPosistion(2.623, 6.996, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            } else {
              swerveDrive.setDesiredPosistion(2.623, 4.104, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
            }
          /* Back to subwoofer
          if (blue){
            swerveDrive.setDesiredPosistion(1.365, 5.548, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
            swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
            swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }*/
          // Step 6 check
          if (Math.abs(shooterAndIntake.returnAngle() - 33) <= 2){
            // if lined up
            shooterAndIntake.angle(0);
            timer.reset();
            autoStep = 7;
          } 
          break;
          case 7:
          // Step seven
          // auto left
          // shoot second note
          shooterAndIntake.intake(1);
          shooterAndIntake.shooter(0.7);
          // Step 7 check
          if (timer.get() >= 2){
            shooterAndIntake.angle(0);
            shooterAndIntake.shooter(0);
            shooterAndIntake.intake(0);
            autoStep = 8;
          }
          break;
          case 8:
          // Step eight
          // auto left
          if (blue){
              swerveDrive.setDesiredPosistion(2.623, 6.996, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          } else {
              swerveDrive.setDesiredPosistion(2.623, 4.104, 31);
              swerveDrive.driveToPositionTwo(0, 0, 0, gyro.getYaw() + yawOffset);
          }
        }
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
    
    if (joystick.getRawButton(1)){
      swerveDrive.setDesiredPosistion(1, 1, 0);
      swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
    } else if (controller.getXButton()){
      // if override pressed
      swerveDrive.drive(controller.getLeftX(), controller.getLeftY(), controller.getRightX(), gyro.getYaw() + yawOffset);

    //} else if (controller.getBButton()){
    
      // line up robot angle for shooter
      /*swerveDrive.setDesiredYaw(0);
      swerveDrive.drive(joystick.getX(), joystick.getY(), 0, gyro.getYaw() + yawOffset);*/
      // write variable linedUp
    } else if(joystick.getRawButtonPressed(6)){
      // Go to far source
      if (blue){
        swerveDrive.setDesiredPosistion(15.96, 1.274, 120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      } else {
        swerveDrive.setDesiredPosistion(15.96, 6.93, -120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      }
    } else if(joystick.getRawButtonPressed(4)){
      // Go to close source
      if (blue){
        swerveDrive.setDesiredPosistion(14.854, 0.636, 120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      } else {
        swerveDrive.setDesiredPosistion(14.854, 4.568, -120);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      }
    } else if(joystick.getRawButtonPressed(5)){
      // Go to subwoofer
      if (blue){
        swerveDrive.setDesiredPosistion(1.365, 5.48, 0);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      } else {
        swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      }
    } else if(joystick.getRawButtonPressed(3)){
      // Go to amp
      if (blue){
        swerveDrive.setDesiredPosistion(1.842, 7.53, 90);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
      } else {
        swerveDrive.setDesiredPosistion(1.842, 0.451, -90);
        swerveDrive.driveToPosition(0, 0, 0, gyro.getYaw() + yawOffset);
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
    if (controller.getXButton()){
      // secondary driver override
    } else if (joystick.getRawButton(6) || joystick.getRawButton(4)){
      // Source pick up
      shooterAndIntake.setAngle(45);
    } else if(controller.getBButton()){
      // line up shooter for speaker
      shooterAndIntake.setAngle(56);
      //shooterAndIntake.shootInSpeaker(true, botposeInTargetspace);
    } else if(controller.getAButton()){
      // Amp
      shooterAndIntake.setAngle(122);
    } else if(Math.abs(controller.getLeftY()) > 0.02){
      shooterAndIntake.angle(controller.getLeftY());
    } else {
      shooterAndIntake.angle(0);
    }

    // Shooter
    if (joystick.getRawButton(6) || joystick.getRawButton(4)){
      // Source
      shooterAndIntake.shooter(-0.5);
    } else if (controller.getYButton()){
      // Low power or variable shooter
      // 0.15 good low power
      shooterAndIntake.shooter(Preferences.getDouble("ShooterPower", 0.7));
    } else if(controller.getBButton()){
      // Speaker subwoofer
      shooterAndIntake.shooter(0.33);
    } else if (controller.getAButton()){
      // amp shoot
      shooterAndIntake.shooter(0.15);
      // If shooter at angle shoot in amp
      //shooterAndIntake.shootInAmp();
    } else if (controller.getRightTriggerAxis() >= 0.5){
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

    // trap shot
    /*if(joystick.getRawButton(autoStep)){.l
      swerveDrive.setDesiredPosistion(hasTarget, hasTarget, hasTarget);
      swerveDrive.drive(0, 0, 0, gyro.getYaw() + yawOffset);
      if (Math.abs(botposeInTargetspace[1] - 0.2) <= 0.03 && Math.abs(botposeInTargetspace[1] - 0.2) <= 0.03 ){
        shooterAndIntake.trapShot(true);
      } else {
        shooterAndIntake.trapShot(false);
      } 
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

    /*// Intake
    if(controller.getPOV()){

    } else if (controller.getLeftTriggerAxis() >= 0.5){
      shooterAndIntake.intake(1);
      //swerveDrive.setTurnPoint(new Pose2d(0, 0, null)); 
    } else if(controller.getLeftBumper()){
      shooterAndIntake.intakeSpeed(Preferences.getDouble("IntakeTop", 0.7), Preferences.getDouble("IntakeBottom", 0.7));
    }else {
      shooterAndIntake.intake(0);
    }*/
 

    // climber
    /*if (controller.getXButton()){
      // secondary driver override
    } else if (Math.abs(controller.getRightY()) >= 0.02){
      climber.climb(controller.getRightY());
    } else {
      climber.climb(0)
    }*/

    if (wheelJoystick.getRawButtonPressed(2)){
      //bumper
      gyro.reset();
    }

    if (wheelJoystick.getRawButtonPressed(1)){
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
