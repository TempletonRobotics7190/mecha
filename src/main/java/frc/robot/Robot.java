/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode; // disabled, can't find where these are used
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

// for limelight
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * mecha mech robo robot robotic me
 */
public class Robot extends TimedRobot {
  private static final int kFrontRightChannel = 2;
  private static final int kRearRightChannel = 1;
  private static final int kFrontLeftChannel = 4;
  private static final int kRearLeftChannel = 3;

  private static final int kControllerChannel = 0;

  private static final int kLeftShootMotor = 5;
  private static final int kRightShootMotor = 6;

  private static final int kIntakeMotor = 7;
  private static final int kLiftMotor = 8;

  private static final int kLeftBeltMotor = 0;
  private static final int kRightBeltMotor = 1;
  
  // drive motors
  WPI_TalonSRX frontLeft = new WPI_TalonSRX(kFrontLeftChannel);
  WPI_TalonSRX rearLeft = new WPI_TalonSRX(kRearLeftChannel);
  WPI_TalonSRX frontRight = new WPI_TalonSRX(kFrontRightChannel); // this one sucks!
  WPI_TalonSRX rearRight = new WPI_TalonSRX(kRearRightChannel);


  private MecanumDrive m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  // shoot motors
  WPI_VictorSPX leftShoot = new WPI_VictorSPX(kLeftShootMotor);
  WPI_VictorSPX rightShoot = new WPI_VictorSPX(kRightShootMotor);
  SpeedControllerGroup m_shooter = new SpeedControllerGroup(leftShoot, rightShoot);

  // intake and lift motors
  WPI_VictorSPX m_intake = new WPI_VictorSPX(kIntakeMotor);
  WPI_VictorSPX m_lift = new WPI_VictorSPX(kLiftMotor);

  // belt motors
  Spark leftBelt = new Spark(kLeftBeltMotor);
  Spark rightBelt = new Spark(kRightBeltMotor);
  SpeedControllerGroup m_belt = new SpeedControllerGroup(leftBelt, rightBelt);

  private XboxController m_f310 = new XboxController(kControllerChannel); // make sure that the logitech f310 controller is set to xinput mode

  private Timer m_timer = new Timer();
  
  private double[][] instructions = {
    // {x-axis speed, y-axis speed, rotation speed, seconds}
    /**
     * axes (relative to front of robot being north):
     * x: -1.0 south --> +1.0 north
     * y: -1.0 west --> +1.0 east
     * rotation: -1.0 clockwise --> +1.0 counterclockwise
     */
    {-0.5, 0.0, 0.0, 3.0}, // drive west at 0.5 speed for 3.0 seconds
    {0.0, 0.5, 0.1, 2.0}, // drive north at 0.5 speed with 0.1 counterclockwise rotation speed for 2.0 seconds
    {-0.25, 0.25, 0.0, 4.5}, // drive north-west at 0.5 (?) speed for 4.5 seconds
  };

  private int currentInstruction;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    // start at first instruction
    currentInstruction = 0;
  }

  @Override
  public void autonomousPeriodic() {
    double instructionXAxis = 0.0;
    double instructionYAxis = 0.0;
    double instructionRotation = 0.0;
    double instructionSeconds = 1.0;
    if (currentInstruction < instructions.length) {
      instructionXAxis = instructions[currentInstruction][0]; // set y axis value
      instructionYAxis = instructions[currentInstruction][1]; // set x axis value
      instructionRotation = instructions[currentInstruction][2]; // set rotation value
      instructionSeconds = instructions[currentInstruction][3]; // set time for instruction
    }

    if (m_timer.get() < instructionSeconds) {
      m_robotDrive.driveCartesian(instructionXAxis, instructionYAxis, instructionRotation);
    } else {
      m_timer.reset();
      currentInstruction++;
    }
  }

  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.stop();
  }

  @Override
  public void teleopPeriodic() {
    boolean intakeButton = m_f310.getAButton();
    boolean beltButton = m_f310.getXButton();
    boolean revButton = m_f310.getBumper(Hand.kLeft);
    boolean fastButton = m_f310.getBumper(Hand.kRight);
    int reverse = 1;
    double speed = 0.25;
    boolean aimButton = m_f310.getBButton();
    boolean rangeButton = m_f310.getYButton();
    double fireTrigger = m_f310.getTriggerAxis(Hand.kRight);
    int liftPOV = m_f310.getPOV();

    if (revButton) {
      reverse = -1;
    }

    if (fastButton) {
      speed = 0.5;
    }

    double leftx = m_f310.getX(Hand.kLeft)*speed*reverse;
    double lefty = -m_f310.getY(Hand.kLeft)*speed*reverse;
    double rightx = -m_f310.getX(Hand.kRight)*0.6*reverse; // rotation
    m_robotDrive.driveCartesian(leftx, lefty, rightx, 0.0);
    if (Math.abs(leftx) + Math.abs(lefty) + Math.abs(rightx) < 0.08) {
      frontLeft.set(0.05);
      rearLeft.set(0.05);
      frontRight.set(-0.05);
      rearRight.set(-0.05);
    }
    // intake
    if (intakeButton) {
      m_intake.set(0.3 * reverse);
    } else {
      m_intake.set(0);
    }

    // belt
    if (beltButton) {
      m_belt.set(0.5 * reverse);
    } // gets turned off during else statement of shooting sequence

    // lift
    if (liftPOV >= 0) {
      if (liftPOV == 180) {
        m_lift.set(0.25);
      } else if (liftPOV == 0) {
        m_lift.set(-0.25);
      }
    } else {
      m_lift.stopMotor();
    }

    // limelight values
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double v = tv.getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    double kP = 0.1;
    double minCommand = 0.05;
    double steeringAdjust = 0.0;
    double drivingAdjust = 0.0;
    double KpDistance = -0.1;

    if (aimButton && v == 1.0) {
        double headingError = -x; // offset from target to crosshair from -29.8 degrees to 29.8 degrees; may need to switch sign
        if (x > 1.0) {
         steeringAdjust = kP * headingError + minCommand; // may switch sign
        }   else if (x < 1.0) {
          steeringAdjust = kP * headingError - minCommand; // may switch sign
        }
        m_robotDrive.driveCartesian(0.0, 0.0, steeringAdjust);
      // vertical aiming; we will mount camera at 45 degree angle; positive values bring shooter down
    }

    // Limelight seeking
    if (aimButton && v == 0.0) {

      if (v == 0.0)
{
          // We don't see the target, seek for the target by spinning in place at a safe speed.
          steeringAdjust = 0.2;
          m_robotDrive.driveCartesian(0.0, 0.0, steeringAdjust);
}
    }

  // Limelight automatic distancing
  double distanceError = y;
  if (rangeButton)
{
        drivingAdjust = KpDistance * distanceError;

      m_robotDrive.driveCartesian(0.0, drivingAdjust, 0.0);
      }

    // shooting sequence
    if (fireTrigger > 0.75) {
      if (m_timer.get() == 0.0) {
        m_timer.reset();
        m_timer.start();
      }
      if (m_timer.get() > 0.0) {
        if (m_timer.get() < 1.0) {
          m_shooter.set(-1.0*m_timer.get());
        } else {
          m_shooter.set(-1.0);
        }
      }
      if (m_timer.get() > 2.0) {
        m_belt.set(0.7);
      }
    } else {
      m_shooter.stopMotor();
      if (!beltButton) {
        m_belt.set(0);
      }
      m_timer.reset();
      m_timer.stop();
    }
  }
}
