// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.autos.GroundPickupExtension;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.pathfinding.ButtonMapping;
import frc.robot.commands.pathfinding.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.Pivot;

import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  // CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController = new
  // CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  XboxController driverXbox = new XboxController(0);
  Joystick buttonBox = new Joystick(1);

  private final JoystickButton startShooter = new JoystickButton(this.driverXbox, 6);
  private final JoystickButton stopShooter = new JoystickButton(this.driverXbox, 5);
  private final POVButton harvest = new POVButton(this.driverXbox, 0);
  private final POVButton harvestStop = new POVButton(this.driverXbox, -1);
  private final POVButton harvestReverse = new POVButton(this.driverXbox, 180);
  private final JoystickButton shoot = new JoystickButton(this.driverXbox, 1);
  private final JoystickButton indexStop = new JoystickButton(this.driverXbox, 1);
  private final JoystickButton target1 = new JoystickButton(this.buttonBox, 1);
  private final JoystickButton target2 = new JoystickButton(this.buttonBox, 2);
  private final JoystickButton target3 = new JoystickButton(this.buttonBox, 3);
  private final JoystickButton target4 = new JoystickButton(this.buttonBox, 4);
  private final JoystickButton target5 = new JoystickButton(this.buttonBox, 5);
  private final JoystickButton target6 = new JoystickButton(this.buttonBox, 6);
  private final JoystickButton target7 = new JoystickButton(this.buttonBox, 7);
  private final JoystickButton target8 = new JoystickButton(this.buttonBox, 8);
  private final JoystickButton target9 = new JoystickButton(this.buttonBox, 9);
  private final JoystickButton targetingMode = new JoystickButton(this.driverXbox, 3);
  private final POVButton GroundPickupExtension = new POVButton(this.driverXbox, 90);
  private final POVButton HomeExtension = new POVButton(this.driverXbox, 90);
  private final POVButton HomePivot = new POVButton(this.driverXbox, 270);
  private final POVButton GroundPickupPivot = new POVButton(this.driverXbox, 270);

  private final 
  
  Shooter shooter = new Shooter();
  Pivot pivot = new Pivot();
  Extender extender = new Extender();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox::getYButtonPressed,
        driverXbox::getAButtonPressed,
        driverXbox::getXButtonPressed,
        driverXbox::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(4));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(4));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
    //new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));


   //new JoystickButton(driverXbox,
    //   2).whileTrue(
    //       Commands.deferredProxy(() -> drivebase.driveToPose(
    //           new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)));


    //new JoystickButton(driverXbox, 3).whileTrue(new RepeatCommand(new
    //InstantCommand(drivebase::lock, drivebase)));

    this.startShooter.onTrue(new InstantCommand(() -> this.shooter.startShooter(), new Subsystem[0]));
    this.stopShooter.onTrue(new InstantCommand(() -> this.shooter.stopShooter(), new Subsystem[0]));
    this.harvest.onTrue(new InstantCommand(() -> this.shooter.harvest(), new Subsystem[0]));
    this.harvestStop.onTrue(new InstantCommand(() -> this.shooter.harvestStop(), new Subsystem[0]));
    this.harvestReverse.onTrue(new InstantCommand(() -> this.shooter.harvestReverse(), new Subsystem[0]));
    this.shoot.onTrue(new InstantCommand(() -> this.shooter.shoot(), new Subsystem[0]));
    this.indexStop.onFalse(new InstantCommand(() -> this.shooter.indexStop(), new Subsystem[0]));
    this.targetingMode.onTrue(new InstantCommand(() -> Vision.targetingOn(), new Subsystem[0]));
    this.targetingMode.onFalse(new InstantCommand(() -> Vision.targetingOff(), new Subsystem[0]));
    this.GroundPickupExtension.onTrue(new InstantCommand(() -> this.extender.setGroundPose(), new Subsystem[0]));
    this.HomeExtension.onFalse(new InstantCommand(() -> this.extender.setHomeState(), new Subsystem[0]));
    this.GroundPickupPivot.onTrue(new InstantCommand(() -> this.pivot.setGroundPickup(), new Subsystem[0]));
    this.HomePivot.onFalse(new InstantCommand(() -> this.pivot.setHomeState(), new Subsystem[0]));
    this.target1.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(1)));
    this.target2.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(2)));
    this.target3.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(3)));
    this.target4.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(4)));
    this.target5.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(5)));
    this.target6.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(6)));
    this.target7.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(7)));
    this.target8.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(8)));
    this.target9.onTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(9)));

    //this.extender.setDefaultCommand(new InstantCommand(() -> this.extender.extendAmount((float) -this.driverXbox.getRawAxis(5)), extender));
    this.pivot.setDefaultCommand(new InstantCommand(() -> this.pivot.moveAmount((float) -this.driverXbox.getRawAxis(5)), pivot));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Test", true);

  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}