// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// check zeroYaw works
package frc.robot;
// Important !! Bionic Beast

// what is best intake angle

import java.util.Arrays;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// Network table for Limelight
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
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

  boolean inTeleop = false;

  // private final CANCoder AbsoluteEncoder = new CANCoder(0);
  // private WPI_CANCoder CANCoder = new
  // ** */
  /*
   * AHRS gyro = new AHRS(SPI.Port.kMXP);
   * float yawOffset = 0;
   */

  // Joystick
  private XboxController controller = new XboxController(0);
  private Joystick joystick = new Joystick(1);
  private Joystick wheelJoystick = new Joystick(2);

  // ***
  double getJoystickForward() {
    // This method returns a positive value when the joystick is pushed forward
    return -joystick.getY();
  }

  // ***
  double getJoystickSideways() {
    // This method returns a positive value when the joystick is pushed to the left
    return -joystick.getX();
  }

  // ***
  double getSteeringWheelAxis() {
    // This method returns a positive value when the wheel is pushed to the left
    return -wheelJoystick.getRawAxis(0);
  }

  // Shooter and Intake
  ShooterAndIntake shooterAndIntake = new ShooterAndIntake(false, false, false, false, false,
      0, 1);

  // Climber
  Climber climber = new Climber(false, false, true, true);

  // Swerve drive
  private WheelDrive backLeft = new WheelDrive(2, 1, 0);
  private WheelDrive backRight = new WheelDrive(4, 3, 1);
  private WheelDrive frontLeft = new WheelDrive(8, 7, 3);
  private WheelDrive frontRight = new WheelDrive(6, 5, 2);
  private SwerveDrive swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

  // Limelight

  // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  // NetworkTableEntry video = table.getEntry("stream").setNumber(0);
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  // Whether the limelight has any valid targets
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ledMode = table.getEntry("ledMode");
  // ID of primary april tag
  NetworkTableEntry IDNum = table.getEntry("tid");
  long primaryTagID;
  NetworkTableEntry  camerapose_targetspace = table.getEntry("camerapose_targetspace");

  // limelight json
  LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
  // double[] botpose2 = LimelightHelpers.getBotPose("limelight");
  double[] botpose2 = LimelightHelpers.getBotPose("limelight");

  // double[] botposeRed = llresults.results.botpose_wpired;

  double x;
  double y;
  double area;
  double v;
  double hasTarget = 0;
  double ts;
  double botposeRed;
  double botposeBlue;
  double cameraposeInTargetspace[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  SwerveModulePosition[] swervedrivepositions[];
  boolean blue;

  double autoInitialShooterAngle;
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

  double autoShooterAngleSecondShot;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("AutoMiddle", kAutoMiddle);
    m_chooser.addOption("AutoRight", kAutoRight);
    m_chooser.addOption("AutoLeft", kAutoLeft);
    SmartDashboard.putData("Auto choices", m_chooser);

    // CameraServer.startAutomaticCapture();
    shooterAndIntake.setAngleEncoder(20.5);

    // zeros angle encoders
    frontRight.zeroEncoders(0.675 * 360);
    frontLeft.zeroEncoders(0.37 * 360);
    backRight.zeroEncoders(0.4763 * 360);
    backLeft.zeroEncoders(0.275 * 360);

    // Inverts drive motors
    frontRight.invertDriveMotor(false);
    frontLeft.invertDriveMotor(true);
    backRight.invertDriveMotor(false);
    backLeft.invertDriveMotor(true);
    // Inverts angle motors
    frontRight.invertAngleMotor(true);
    frontLeft.invertAngleMotor(true);
    backRight.invertAngleMotor(true);
    backLeft.invertAngleMotor(true);

    swerveDrive.periodicOdometry();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // <RED ACTION>
        blue = false;
        PositionHelpers.setAllianceIsBlue(blue);
      }
      if (ally.get() == Alliance.Blue) {
        // <BLUE ACTION>
        blue = true;
        PositionHelpers.setAllianceIsBlue(blue);
      }
    }

    PositionHelpers.assignSwerve(swerveDrive);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   * s
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    swerveDrive.periodicOdometry();
    SmartDashboard.putBoolean("Blue", blue);
    SmartDashboard.putNumber("X error", swerveDrive.returnXError());
    SmartDashboard.putNumber("Y error", swerveDrive.returnYError());
    // Limelight
    if (blue) {
      botpose2 = LimelightHelpers.getBotPose_wpiBlue("limelight");
      SmartDashboard.putString("botpose2", Arrays.toString(botpose2));

    } else {
      botpose2 = LimelightHelpers.getBotPose_wpiRed("limelight");
      SmartDashboard.putString("botpose2", Arrays.toString(botpose2));
    }
    if (hasTarget == 1 && inTeleop) {
      if (botpose2.length < 6) {
        System.out.print(botpose2);
        botpose2 = new double[6];
        System.out.println(botpose2);
        Arrays.fill(botpose2, 0);
      }
      if (cameraposeInTargetspace.length < 6) {
        cameraposeInTargetspace = new double[6];
        Arrays.fill(cameraposeInTargetspace, 0);
      }
      SwerveModulePosition[] modulePositions = { frontLeft.getPosition(),
          frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition() };
      swerveDrive.resetPosition(modulePositions,
        new Pose2d(cameraposeInTargetspace[0], cameraposeInTargetspace[2], new Rotation2d(-(cameraposeInTargetspace[4] / 360 * 2 * Math.PI))),
          new Pose2d(botpose2[0], botpose2[1], new Rotation2d(-(botpose2[5] / 360 * 2 * Math.PI))), hasTarget);
    }
    cameraposeInTargetspace = camerapose_targetspace.getDoubleArray(cameraposeInTargetspace);

    SmartDashboard.putNumberArray("Camera PoseInTargetspace", camerapose_targetspace.getDoubleArray(cameraposeInTargetspace));
    SmartDashboard.putNumber("robot from target yaw", cameraposeInTargetspace[4]);
    SmartDashboard.putString("cameraposeInTargetspace", Arrays.toString(cameraposeInTargetspace));
    /* // angle motors
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
    // speed motors
    // Relative Encoders
    SmartDashboard.putNumber("backLeft drive encoder", backLeft.returnDrivePosition());
    SmartDashboard.putNumber("backRight drive encoder", backRight.returnDrivePosition());
    SmartDashboard.putNumber("frontRight drive encoder", frontRight.returnDrivePosition());
    SmartDashboard.putNumber("frontLeft drive encoder", frontLeft.returnDrivePosition()); */

    // Gyro angle
    SmartDashboard.putNumber("Yaw", swerveDrive.getGyroRobotYaw());

    // Color sensor
    shooterAndIntake.updateDashboardForColorSensor();

    // Drive method
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());

    /* SmartDashboard.putNumber("forward", swerveDrive.returnForward());
    SmartDashboard.putNumber("strafe", swerveDrive.returnStrafe());
    SmartDashboard.putNumber("turning", swerveDrive.returnTurning()); */

    //
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    SmartDashboard.putNumber("desiredX", swerveDrive.returnDesiredX());
    SmartDashboard.putNumber("desiredY", swerveDrive.returnDesiredY());
    SmartDashboard.putNumber("desiredYaw", swerveDrive.returnDesiredYaw());
    SmartDashboard.putNumber("currentAngle", backRight.returncurrentAngle());

    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    hasTarget = tv.getDouble(0);
    SmartDashboard.putNumber("Has target", hasTarget);

    // Gets the value of the primary april tag we are detecting
    primaryTagID = IDNum.getInteger(0);
    // SmartDashboard.putNumber("April tag ID", primaryTagID);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);


    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("Shooter Angle encoder", shooterAndIntake.returnAngle());

    SmartDashboard.putString("rotation2d", swerveDrive.returnRotation().toString());
    SmartDashboard.putNumber("robot X", swerveDrive.returnX());
    SmartDashboard.putNumber("robot Y", swerveDrive.returnY());

    SmartDashboard.putNumber("Speaker distance", PositionHelpers.getSpeakerDistance());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    inTeleop = false;
    timer.stop();
    timer.start();
    autoStep = 1;
    // autoStep = 0;
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // <RED ACTION>
        blue = false;
        PositionHelpers.setAllianceIsBlue(blue);
      }
      if (ally.get() == Alliance.Blue) {
        // <BLUE ACTION>
        blue = true;
        PositionHelpers.setAllianceIsBlue(blue);
      }
    }
    swerveDrive.zeroGyro();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    // Sets positions on the field based on our auto selected and alliance color
    switch (m_autoSelected) {
      case kAutoMiddle:
        autoShooterAngleSecondShot = 37;
        if (blue) {
          autoStartingPositionX = 1.365;
          autoStartingPositionY = 5.548;
          autoStartingPositionRotation = 0;
          autoInitialShooterAngle = 57;
          // Middle position doesn't have a second position

          //autoThirdPositionX = 2.623;
          autoThirdPositionX = 2.769;
          autoThirdPositionY = 5.548;
          autoThirdPositionRotation = 0;

          autoShootingThirdPositionX = 2.623;
          autoShootingThirdPositionY = 5.548;
          autoShootingThirdPositionRotation = 0;
        } else {
          autoStartingPositionX = 1.365;
          autoStartingPositionY = 2.656;
          autoStartingPositionRotation = 0;
          autoInitialShooterAngle = 57;
          // Middle position doesn't have a second position

          //autoThirdPositionX = 2.623;
          autoThirdPositionX = 2.769;
          autoThirdPositionY = 2.656;
          autoThirdPositionRotation = 0;

          autoShootingThirdPositionX = 2.623;
          autoShootingThirdPositionY = 2.656;
          autoShootingThirdPositionRotation = 0;
        }
        break;
      case kAutoRight:
        autoShooterAngleSecondShot = 37;
        if (blue) {
          autoStartingPositionX = 0.683;
          autoStartingPositionY = 4.365;
          autoStartingPositionRotation = -60;
          autoInitialShooterAngle = 60;

          //autoSecondPositionX = 0.836;
          autoThirdPositionX = 2.769;
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
          autoInitialShooterAngle = 60;

          //autoSecondPositionX = 0.836;
          autoThirdPositionX = 2.769;
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
        autoShooterAngleSecondShot = 37;
        if (blue) {
          autoStartingPositionX = 0.683;
          autoStartingPositionY = 6.731;
          autoStartingPositionRotation = 60;
          autoInitialShooterAngle = 60;

          //autoSecondPositionX = 0.836;
          autoSecondPositionX = 0.836 + 0.3048;
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
          autoInitialShooterAngle = 60;

          //autoSecondPositionX = 0.836;
          autoThirdPositionX = 2.769;
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
    swerveDrive.setYawOffset(autoStartingPositionRotation);
    // yawOffset = (float) autoStartingPositionRotation;
    swerveDrive.setPosition(swerveDrive.getGyroRobotYaw(),
        new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition() },
        new Pose2d(autoStartingPositionX, autoStartingPositionY,
            new Rotation2d(swerveDrive.degreesToRadians(autoStartingPositionRotation))));

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_autoSelected = m_chooser.getSelected();
    SmartDashboard.putString("Auto selected", m_autoSelected);

    // If default is selected, don't do anything
    if (m_autoSelected == kDefaultAuto) {
      return;
    }

    switch (autoStep) {
      case 1:
        // Step one
        // set shooter angle and speed
        shooterAndIntake.setAngle(autoInitialShooterAngle);
        shooterAndIntake.shooterToSpeed(0.33);
        // Step 1 check
        if (timer.get() >= 3) {
          // if (Math.abs(shooterAndIntake.returnAngle() - 57) <= 2) {
          // if (timer.get() >= 3){
          // if shooter angle is close to 3
          System.out.println("Start of step 2");
          shooterAndIntake.angle(0);
          timer.reset();
          autoStep = 2;
        }
        break;
      case 2:
        // Step two
        // shoot first note in speaker
        shooterAndIntake.intake(1);
        shooterAndIntake.shooter(0.33);
        // Step 2 check
        if (timer.get() >= 2) {
          System.out.println("Start of step 3");
          // if 2 or more seconds on step 2
          shooterAndIntake.shooter(0);
          shooterAndIntake.intake(0);
          autoStep = 3;
        }
        break;
      case 3:
        // Step three
        // drive to second position
        // middle doesn't have 2nd position
        if (m_autoSelected == kAutoMiddle) {
          System.out.println("Start of step 4");
          // if at 2nd position
          autoStep = 4;
          break;
        }
        // set shooter angle to intake better
        shooterAndIntake.setAngle(25);
        // drive to second position
        swerveDrive.setDesiredPosistion(autoSecondPositionX, autoSecondPositionY, autoSecondPositionRotation);
        swerveDrive.driveToPositionTwo();
        // Step 3 check
        if (Math.hypot(autoSecondPositionX - swerveDrive.returnX(),
            autoSecondPositionY - swerveDrive.returnY()) <= 0.09) {
          System.out.println("Start of step 4");
          // if at 3rd position
          autoStep = 4;
        }
        break;
      case 4:
        // Step four
        // 3rd position
        // intake second note
        shooterAndIntake.setAngle(25);
        shooterAndIntake.intake(1);
        //System.out.println(blue ? "blue" : "red");
        // drive to note
        swerveDrive.setDesiredPosistion(autoThirdPositionX, autoThirdPositionY, autoThirdPositionRotation);
        swerveDrive.driveToPositionTwo();
        // Step 4 check
        if (Math.hypot(autoThirdPositionX - swerveDrive.returnX(),
            autoThirdPositionY - swerveDrive.returnY()) <= 0.05) {
          // if at 3rd position
          System.out.println("Start of step 5 second shot");
          swerveDrive.driveTeleop(0, 0, 0);
          shooterAndIntake.intake(0);
          timer.reset();
          autoStep = 5;
        }
        break;

      case 5:
      // step 5
      // pull second note back
        shooterAndIntake.intake(-1);
        //shooterAndIntake.setAngle(autoShooterAngleSecondShot);
        shooterAndIntake.setAngleForSpeaker();
        swerveDrive.setDesiredPosistion(autoShootingThirdPositionX, autoShootingThirdPositionY,
            autoShootingThirdPositionRotation);
        swerveDrive.driveToPositionTwo();
        if (timer.get() >= 0.5) {
          // step 5 check
          System.out.println("Start of step 6");
          shooterAndIntake.intake(0);
          autoStep = 6;
        }
        break;
      case 6:
        // Step six
        // line up to shoot second note
        // power up shooter
        //shooterAndIntake.shooter(0.7);
        //TODO
        shooterAndIntake.rampForSpeaker();
        shooterAndIntake.setAngleForSpeaker();
        //swerveDrive.pointToSpeaker();
        // set shooter angle
        // TODO test autoShooterAngleSecondShot (33, 37)
        //shooterAndIntake.setAngle(autoShooterAngleSecondShot);
        swerveDrive.setDesiredPosistion(autoShootingThirdPositionX, autoShootingThirdPositionY,
            autoShootingThirdPositionRotation);
        swerveDrive.driveToPositionTwo();

        // Step 6 check
        if (Math.abs(shooterAndIntake.returnAngle() - autoShooterAngleSecondShot) <= 2) {
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
        //shooterAndIntake.shooter(0.7);
        shooterAndIntake.rampForSpeaker();
        shooterAndIntake.setAngleForSpeaker();
        
        // Step 7 check
        if (timer.get() >= 2) {
          System.out.println("Start of step 8");
          shooterAndIntake.angle(0);
          shooterAndIntake.shooter(0);
          shooterAndIntake.intake(0);
          autoStep = 8;
        }
        break;
      case 8:
        // Step eight
        // drive towards center line
        if (blue){
          // we dont want to hit the podium so we need a different position
          if (m_autoSelected == kAutoRight){
            break;
          }
        } else {
          if (m_autoSelected == kAutoLeft){
            break;
          }
        }
        // drive towards center line
        swerveDrive.setDesiredPosistion(autoShootingThirdPositionX + 1, autoShootingThirdPositionY,0);
        swerveDrive.driveToPositionTwo();
        break;
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    swerveDrive.setDesiredYaw(swerveDrive.getGyroRobotYaw());

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // <RED ACTION>
        blue = false;
        PositionHelpers.setAllianceIsBlue(blue);
      }
      if (ally.get() == Alliance.Blue) {
        // <BLUE ACTION>
        blue = true;
        PositionHelpers.setAllianceIsBlue(blue);
      }
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    inTeleop = true;
    if (joystick.getRawButton(1)) {
      swerveDrive.setDesiredPosistion(1, 1, 0);
      swerveDrive.driveToPosition();
    } else if (controller.getXButton()) {
      // if override pressed
      swerveDrive.driveTeleop(controller.getLeftY(), controller.getLeftX(), controller.getRightX());

      // } else if (controller.getBButton()){

      // line up robot angle for shooter
      /*
       * swerveDrive.setDesiredYaw(0);
       * swerveDrive.drive(joystick.getX(), joystick.getY(), 0);
       */
      // write variable linedUp
    } else if (joystick.getRawButton(6)) {
      // Go to far source
      if (blue) {
        swerveDrive.setDesiredPosistion(15.96, 1.274, 120);
        swerveDrive.driveToPosition();
      } else {
        swerveDrive.setDesiredPosistion(15.96, 6.93, -120);
        swerveDrive.driveToPosition();
      }
    } else if (false) { // Currently not being used
      // Go to close source
      if (blue) {
        swerveDrive.setDesiredPosistion(14.854, 0.636, 120);
        swerveDrive.driveToPosition();
      } else {
        swerveDrive.setDesiredPosistion(14.854, 4.568, -120);
        swerveDrive.driveToPosition();
      }
    } else if (joystick.getRawButton(4)) {
      // Setup for a long speaker shot
      swerveDrive.pointToSpeaker();
    } else if (joystick.getRawButton(5)) {
      // Go to subwoofer
      if (blue) {
        swerveDrive.setDesiredPosistion(1.365, 5.48, 0);
        swerveDrive.driveToPosition();
      } else {
        swerveDrive.setDesiredPosistion(1.365, 2.656, 0);
        swerveDrive.driveToPosition();
      }
    } else if (joystick.getRawButton(3)) {
      // Go to amp
      if (blue) {
        swerveDrive.setDesiredPosistion(1.842, 7.53, 90);
        swerveDrive.driveToPosition();
      } else {
        swerveDrive.setDesiredPosistion(1.842, 0.451, -90);
        swerveDrive.driveToPosition();
      }
    } else if (joystick.getPOV() == 0) {
      // North
      swerveDrive.setDesiredYaw(0);
      swerveDrive.driveTeleop(getJoystickForward(), getJoystickSideways(), 0);
    } else if (joystick.getPOV() == 180) {
      // South
      swerveDrive.setDesiredYaw(180);
      swerveDrive.driveTeleop(getJoystickForward(), getJoystickSideways(), 0);
    } else if (joystick.getPOV() == 90) {
      // East
      swerveDrive.setDesiredYaw(-90);
      swerveDrive.driveTeleop(getJoystickForward(), getJoystickSideways(), 0);
    } else if (joystick.getPOV() == 270) {
      // West
      swerveDrive.setDesiredYaw(90);
      swerveDrive.driveTeleop(getJoystickForward(), getJoystickSideways(), 0);

    } /*
       * else if (joystick.getRawButton(11)){
       * shooterAndIntake.setAngle(13);
       * 
       * }
       */
    else {
      // primary driver inputs
      swerveDrive.driveTeleop(getJoystickForward(), getJoystickSideways(), getSteeringWheelAxis());
    }

    // Shooter angle
    if (joystick.getRawButton(4)) {
      // Setup for a long speaker shot
      shooterAndIntake.setAngleForSpeaker();
    } else if (controller.getXButton()) {
      // secondary driver override
    } else if (joystick.getRawButton(6)) {
      // Source pick up
      shooterAndIntake.setAngle(45);
    } else if (controller.getYButton()) {
      // source pick up controller
      // shooterAndIntake.setAngle(Preferences.getDouble("Source Angle", 45));
      shooterAndIntake.setAngle(56);
    } else if (controller.getBButton()) {
      // line up shooter for speaker
      shooterAndIntake.setAngle(60);
      // shooterAndIntake.shootInSpeaker(true, cameraposeInTargetspace);
    } else if (controller.getAButton()) {
      // Amp
      // shooterAndIntake.setAngle(116.0);
      shooterAndIntake.setAngle(Preferences.getDouble("Amp Angle", 45));
    } else if (Math.abs(controller.getLeftY()) > 0.02) {
      shooterAndIntake.angle(controller.getLeftY());
    } else {
      shooterAndIntake.angle(0);
    }

    // Shooter
    if (joystick.getRawButton(6)) {
      // Source
      shooterAndIntake.shooter(-0.5);
    } else if (controller.getYButton()) {
      // source
      shooterAndIntake.shooter(-0.5);
      // Low power or variable shooter
      // 0.15 good low power
      // shooterAndIntake.shooter(Preferences.getDouble("ShooterPower", 0.7));
    } else if (controller.getBButton()) {
      // Speaker subwoofer
      shooterAndIntake.shooter(0.33);
    } else if (controller.getAButton()) {
      // amp shoot
      // shooterAndIntake.shooter(Preferences.getDouble("Amp power", 0.15));
      shooterAndIntake.shooter(0.0825);
      // If shooter at angle shoot in amp
      // shooterAndIntake.shootInAmp();
    } else if (controller.getPOV() == 0) {
      // top
      // Long range speaker shot
      shooterAndIntake.rampForSpeaker();
    } else if (controller.getPOV() == 180) {
      // bottom
      shooterAndIntake.shooter(0.2);
    } else if (controller.getRightTriggerAxis() >= 0.5) {
      // Shoot out
      shooterAndIntake.shooter(controller.getRightTriggerAxis());
    } else if (controller.getRightBumper()) {
      // Reverse shooter
      shooterAndIntake.shooter(-0.25);
    } else {
      shooterAndIntake.shooter(0);
    }

    /*
     * (if ( controller.getRightTriggerAxis() >= 0.5){
     * shooterAndIntake.shooter(controller.getRightTriggerAxis());
     * } else if(controller.getRightBumper()){
     * shooterAndIntake.shooter(-0.5);
     * } else {
     * shooterAndIntake.shooter(0);
     * }
     */

    // trap shot
    if (joystick.getRawButton(12)) {

      switch ((int) primaryTagID) {
        case 15:
        case 11:
          // left stage
          swerveDrive.setDesiredPosistion(4.248, 5.18, 120);
          swerveDrive.driveToPosition();
          // If lined up at trap
          if (swerveDrive.compareFieldPositions(4.248, 5.18, swerveDrive.returnX(),
              swerveDrive.returnY()) <= 0.03) {
            // shoot into trap
            shooterAndIntake.trapShot(true);
          } else {
            // set correct shooter speed and angle
            shooterAndIntake.trapShot(false);
          }
          break;
        case 14:
        case 13:
          // far middle
          swerveDrive.setDesiredPosistion(6.108, 4.105, 0);
          swerveDrive.driveToPosition();
          // If lined up at trap
          if (swerveDrive.compareFieldPositions(6.108, 4.105, swerveDrive.returnX(),
              swerveDrive.returnY()) <= 0.03) {
            // shoot into trap
            shooterAndIntake.trapShot(true);
          } else {
            // set correct shooter speed and angle
            shooterAndIntake.trapShot(false);
          }
          break;
        case 16:
        case 12:
          // right
          swerveDrive.setDesiredPosistion(4.248, 3.031, -120);
          swerveDrive.driveToPosition();
          // If lined up at trap
          if (swerveDrive.compareFieldPositions(4.248, 3.031, swerveDrive.returnX(),
              swerveDrive.returnY()) <= 0.03) {
            // shoot into trap
            shooterAndIntake.trapShot(true);
          } else {
            // set correct shooter speed and angle
            shooterAndIntake.trapShot(false);
          }
          break;
        default:
          // default
          shooterAndIntake.trapShot(false);
          break;

      }
    }

    // Intake
    if (controller.getLeftTriggerAxis() >= 0.5) {
      shooterAndIntake.intake(1);
      // swerveDrive.setTurnPoint(new Pose2d(0, 0, null));
    } else if (controller.getLeftBumper()) {
      shooterAndIntake.intake(-1);
      // shooterAndIntake.intakeSpeed(Preferences.getDouble("IntakeTop", 0.7),
      // Preferences.getDouble("IntakeBottom", 0.7));
    } else {
      shooterAndIntake.intake(0);
    }

    // climber
    if (controller.getXButton()) {
      // secondary driver override
    } else if ((Math.abs(controller.getRightY()) >= 0.02) && controller.getPOV() == 90) {
      climber.rightClimber(controller.getRightY());
    } else if ((Math.abs(controller.getRightY()) >= 0.02) && controller.getPOV() == 270) {
      climber.leftClimber(controller.getRightY());
    } else if (Math.abs(controller.getRightY()) >= 0.02) {
      climber.climb(controller.getRightY());
    } else {
      climber.climb(0);
    }

    if (wheelJoystick.getRawButtonPressed(2)) {
      // bumper
      swerveDrive.zeroGyro();
    }

    if (wheelJoystick.getRawButtonPressed(1)) {
      //
      // Inverts drive motors
      frontRight.invertDriveMotor(false);
      frontLeft.invertDriveMotor(true);
      backRight.invertDriveMotor(false);
      backLeft.invertDriveMotor(true);
      // Inverts angle motors
      frontRight.invertAngleMotor(true);
      frontLeft.invertAngleMotor(true);
      backRight.invertAngleMotor(true);
      backLeft.invertAngleMotor(true);
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    inTeleop = false;
    swerveDrive.offDrive();
    shooterAndIntake.offShooterAndIntake();
    climber.offClimber();
    swerveDrive.setDesiredYaw(swerveDrive.getGyroRobotYaw());
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
