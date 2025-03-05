package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.REVLibError;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.*;




public class AlgaeSubsystem extends SubsystemBase {
    
    private final SparkMax AlgaeMotor = new SparkMax(Constants.IDConstants.Algae_Intake_ID, MotorType.kBrushless);
    private SparkMax                PivotMotor;
    private  ProfiledPIDController  mWristPIDController;
    private  ArmFeedforward         mWristFeedForward;
    private  Encoder mWristAbsEncoder = new Encoder(Constants.Coral_Algae_Constants.mWristEncoderID1,Constants.Coral_Algae_Constants.mWristEncoderID2,Constants.Coral_Algae_Constants.mWristEncoderInvert);
    private PeriodicIO mPeriodicIO;
    
    public AlgaeSubsystem() {
      
      mPeriodicIO = new PeriodicIO();

      PivotMotor = new SparkMax(Constants.IDConstants.Algae_Pivot_ID, MotorType.kBrushless);
      SparkMaxConfig wristConfig = new SparkMaxConfig();
      wristConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(Constants.Coral_Algae_Constants.kMaxWristCurrent)
          .inverted(true);
          
  
      

          mWristPIDController = new ProfiledPIDController(
        Constants.Coral_Algae_Constants.kWristP,
        Constants.Coral_Algae_Constants.kWristI,
        Constants.Coral_Algae_Constants.kWristD,
        new TrapezoidProfile.Constraints(
            Constants.Coral_Algae_Constants.kWristMaxVelocity,
            Constants.Coral_Algae_Constants.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        Constants.Coral_Algae_Constants.kWristKS,
        Constants.Coral_Algae_Constants.kWristKG,
        Constants.Coral_Algae_Constants.kWristKV,
        Constants.Coral_Algae_Constants.kWristKA);


          // INTAKE
    
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    
    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Coral_Algae_Constants.kMaxIntakeCurrent)
        .inverted(true);

      
   
    }

    

    private static class PeriodicIO {
      double wrist_target_angle = 0.0;
      double wrist_voltage = 0.0;
      double intake_power = 0.0;
      IntakeState state = IntakeState.STOW;
  }


  

    public enum IntakeState {
        NONE,
        STOW,
        DEALGAE,
        GROUND
      }
 
      
      public void periodic() {
          double pidCalc = mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle);
          double ffCalc = mWristFeedForward.calculate(
              Math.toRadians(getWristReferenceToHorizontal()),
              Math.toRadians(mWristPIDController.getSetpoint().velocity)
          );
  
          mPeriodicIO.wrist_voltage = pidCalc + ffCalc;
      }
  
      public void setWristVoltage(double voltage) {
          AlgaeMotor.set(voltage);
      }
  
      public void setIntakePower(double power) {
          AlgaeMotor.set(power);
      }
  
      public double getWristAngle() {
          return Units.rotationsToDegrees(mWristAbsEncoder.get());
      }
  
      public double getWristReferenceToHorizontal() {
          return getWristAngle() - Constants.Coral_Algae_Constants.kWristOffset;
      }
  
      public IntakeState getState() {
          return mPeriodicIO.state;
      }
  
      public void stop() {
          mPeriodicIO.wrist_voltage = 0.0;
          mPeriodicIO.wrist_target_angle = Constants.Coral_Algae_Constants.kStowAngle;
  
          PivotMotor.set(0.0);
          AlgaeMotor.set(0.0);
      }
  
      // Custom actions such as stow, grabAlgae, etc., can be executed from commands.
      public void stow() {
          mPeriodicIO.wrist_target_angle = Constants.Coral_Algae_Constants.kStowAngle;
          mPeriodicIO.state = IntakeState.STOW;
      }
  
      public void grabAlgae() {
          mPeriodicIO.wrist_target_angle = Constants.Coral_Algae_Constants.kDeAlgaeAngle;
          mPeriodicIO.intake_power = Constants.Coral_Algae_Constants.kIntakeSpeed;
          mPeriodicIO.state = IntakeState.DEALGAE;
      }
  
      public void score() {
          mPeriodicIO.intake_power = mPeriodicIO.state == IntakeState.GROUND
              ? -Constants.Coral_Algae_Constants.kEjectSpeed
              : Constants.Coral_Algae_Constants.kEjectSpeed;
      }
  
      public void groundIntake() {
          mPeriodicIO.wrist_target_angle = Constants.Coral_Algae_Constants.kGroundIntakeAngle;
          mPeriodicIO.intake_power = Constants.Coral_Algae_Constants.kGroundIntakeSpeed;
          mPeriodicIO.state = IntakeState.GROUND;
      }

       public Command setAngleGoal(double goal)
  {
    return startRun(()->{mWristPIDController.reset(getWristAngle());},() -> reachAngleGoal(goal));
  }

   public void reachAngleGoal(double goal)
  {
    double voltsOut = MathUtil.clamp(
        mWristPIDController.calculate(getWristAngle(), goal) +
        mWristFeedForward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              mWristPIDController.getSetpoint().velocity),
        -12,
        12); // 7 is the max voltage to send out.
    PivotMotor.setVoltage(voltsOut);
  }

  public double getVelocityMetersPerSecond()
  {
    return ((mWristAbsEncoder.getDistance() / 60) / ElevatorConstants.kElevatorGearing) *
           (2 * Math.PI * ElevatorConstants.kElevatorDrumRadius);
  }
  
  public Command Stow(){
return 
      }
}
