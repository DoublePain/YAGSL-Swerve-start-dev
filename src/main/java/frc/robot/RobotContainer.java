/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GrabAlgaeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.StopAlgaeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.StopCoralCommand;
import frc.robot.commands.StowCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

// This class is where the bulk of the robot should be declared.  Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the Robot
// periodic methods (other than the scheduler calls).  Instead, the structure of the robot
// (including subsystems, commands, and button mappings) should be declared here.
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_driverController2 = new CommandXboxController(OperatorConstants.kDriverControllerPort2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

     SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                        () -> m_driverController.getLeftY() * .5,
                                                        () -> m_driverController.getLeftX() * .5)
                                                      .withControllerRotationAxis(m_driverController::getRightX)
                                                      .scaleRotation(0.5)
                                                      .deadband(OperatorConstants.deadband)
                                                      .scaleTranslation(0.5)
                                                      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
  m_driverController::getRightY).headingWhile(true);
  
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2)*0.5)
                                                                    .deadband(OperatorConstants.deadband)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);
  
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    /*NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("Elevator L4",  new ElevatorSubsystem().setElevatorHeight(.635) );
    NamedCommands.registerCommand("Elevator L1",  new ElevatorSubsystem().setElevatorHeight(0.175) );
    NamedCommands.registerCommand("Elevator L3",  new ElevatorSubsystem().setElevatorHeight(0.315) );
    NamedCommands.registerCommand("Elevator stow",  new ElevatorSubsystem().setElevatorHeight(-0.05) );
    NamedCommands.registerCommand("Intake Coral", new IntakeCommand(m_coralSubsystem));
    NamedCommands.registerCommand("Reverse Coral", new ReverseCommand(m_coralSubsystem));
    NamedCommands.registerCommand("Stop coral", new StopCoralCommand(m_coralSubsystem));
    NamedCommands.registerCommand("Grab algae", new GrabAlgaeCommand(m_algaeSubsystem));
    NamedCommands.registerCommand("Stop algae", new StopAlgaeCommand(m_algaeSubsystem));
    NamedCommands.registerCommand("Stop coral", new StopCoralCommand(m_coralSubsystem));
    NamedCommands.registerCommand("Stow algae", new StowCommand(m_algaeSubsystem));
    NamedCommands.registerCommand("Score L1", new ScoreCoralCommand(m_coralSubsystem, false));
    NamedCommands.registerCommand("Score high", new ScoreCoralCommand(m_coralSubsystem, true));
    */

    CanBridge.runTCP();


    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.hold());
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.setPower(0));
    // m_driverController.a().whileTrue(m_elevatorSubsystem.runSysIdRoutine());
   // m_driverController.x().whileTrue(m_elevatorSubsystem.setPower(-0.1));
   // m_driverController.y().whileTrue(m_elevatorSubsystem.setPower(0.1));

   boolean elevatorTesting = false;

   if(elevatorTesting)
   {
    m_driverController2.x().whileTrue(m_elevatorSubsystem.CoralL4().repeatedly());  // L4
    m_driverController2.b().whileTrue(m_elevatorSubsystem.CoralL3().repeatedly());  // L3
    m_driverController2.a().whileTrue(m_elevatorSubsystem.CoralL2().repeatedly());  // L2
    m_driverController2.y().whileTrue(m_elevatorSubsystem.CoralL1().repeatedly());  // L1

    m_driverController2.leftBumper().whileTrue(m_elevatorSubsystem.AlgaeL23().repeatedly());  // L1
    m_driverController2.rightBumper().whileTrue(m_elevatorSubsystem.AlgaeL34().repeatedly());  // L1 
   }

   boolean armTesting = true;
   if(armTesting)
   {
    m_elevatorSubsystem.setDefaultCommand(m_elevatorSubsystem.setGoal(0.3));

    m_driverController.y().whileTrue(m_algaeSubsystem.setPower(0.5));
    m_driverController.x().whileTrue(m_algaeSubsystem.setPower(-0.5));
    m_driverController.a().whileTrue(m_algaeSubsystem.runSysIdRoutine());
    m_driverController.b().whileTrue(m_algaeSubsystem.setGoal(0));
    m_driverController.leftBumper().whileTrue(m_algaeSubsystem.setGoal(45));
    m_driverController.rightBumper().whileTrue(m_algaeSubsystem.setGoal(-45));


   }


    

    // configureBindings();

    //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  // Trigger & Button Bindings!
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    SmartDashboard.putData("Side View", Constants.sideView);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
 
  m_driverController2.button(6).whileTrue( new GrabAlgaeCommand(m_algaeSubsystem));  // Button 0 for GrabAlgaeCommand
  m_driverController2.button(6).whileFalse(new StopAlgaeCommand(m_algaeSubsystem));  // Button 0 for StopAlgaeCommand

  m_driverController2.button(5).whileTrue(new ScoreCoralCommand(m_coralSubsystem,true)); 
  m_driverController2.button(1).whileTrue(new ScoreCoralCommand(m_coralSubsystem,false)); 

  m_driverController2.button(1).whileFalse(new StopCoralCommand(m_coralSubsystem)).and
  (m_driverController2.button(5).whileFalse(new StopCoralCommand(m_coralSubsystem)));  

  m_driverController2.button(2).whileTrue(m_elevatorSubsystem.setElevatorHeight(0.175));
  m_driverController2.button(4).whileTrue(m_elevatorSubsystem.setElevatorHeight(0.315));
  m_driverController2.button(3).whileTrue(m_elevatorSubsystem.setElevatorHeight(.635));
  m_driverController2.button(0).whileTrue(m_elevatorSubsystem.setElevatorHeight(-0.05));
  //m_driverController.button(1).whileTrue(driveFieldOrientedDirectAngleKeyboard);
  
  }
  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("MID1");
  }
}
