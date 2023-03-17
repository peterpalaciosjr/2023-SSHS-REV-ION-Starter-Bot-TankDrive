// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CloseGripperAutoCmd;
import frc.robot.commands.OpenGripperAutoCmd;
import frc.robot.commands.ScoringPosAutoCmd;
import frc.robot.commands.HomePosAutoCmd;
import frc.robot.commands.SequentialCommandGroup;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final GripperSubsystem m_gripper = new GripperSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  public static final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();

  public static final XboxController m_driveController = new XboxController(Constants.OIConstants.kDriverController); 
  public static final XboxController m_ArmController = new XboxController(Constants.OIConstants.kArmController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings(); 

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //set up gripper open/close
    new JoystickButton(m_ArmController, XboxController.Button.kRightBumper.value)
      .onTrue(new InstantCommand(() -> m_gripper.openGripper()));
      .onFalse(new InstantCommand(() -> m_gripper.closeGripper()));

    //set up arm preset positions
    new JoystickButton(m_ArmController, XboxController.Button.kA.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kHomePosition, m_gripper)));
    new JoystickButton(m_ArmController, XboxController.Button.kX.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition, m_gripper)));
    new JoystickButton(m_ArmController, XboxController.Button.kY.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition, m_gripper)));
    new JoystickButton(m_ArmController, XboxController.Button.kB.value)
      .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kFeederPosition, m_gripper)));

    //set up arm manual and auto functions
    m_arm.setDefaultCommand(new RunCommand(
      () ->
        m_arm.runAutomatic()
      , m_arm)
    );
    new Trigger(() -> 
      Math.abs(m_ArmController.getRightTriggerAxis() - m_ArmController.getLeftTriggerAxis()) > Constants.OIConstants.kArmManualDeadband
      ).whileTrue(new RunCommand(
        () ->
          m_arm.runManual((m_ArmController.getRightTriggerAxis() - m_ArmController.getLeftTriggerAxis()) * Constants.OIConstants.kArmManualScale)
        , m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
                new ScoringPosAutoCmd(),
                new OpenGripperPAutoCmd(),
                new CloseGripperAutoCmd(),
                new HomePosAutoCmd()

        );
    }
}
