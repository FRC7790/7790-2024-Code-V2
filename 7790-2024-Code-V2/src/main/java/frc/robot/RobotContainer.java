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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CommandFactory;
//import frc.robot.commands.autos.GroundPickupExtension;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.pathfinding.Aiming;
import frc.robot.commands.pathfinding.ButtonMapping;
import frc.robot.commands.pathfinding.Vision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.UnicornHorn;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  XboxController alternate = new XboxController(2);
  Joystick buttonBox2 = new Joystick(3);

   private final JoystickButton startShooter = new
   JoystickButton(this.driverXbox, 3);

  private final JoystickButton harvestManual = new JoystickButton(driverXbox, 1);
  private final JoystickButton harvestManualReverse = new JoystickButton(driverXbox, 2);

  private final JoystickButton harvest = new JoystickButton(this.driverXbox, 5);

  // private final POVButton harvestReverse = new POVButton(this.driverXbox, 180);
  private final JoystickButton shoot = new JoystickButton(this.driverXbox, 6);
  // private final JoystickButton indexStop = new JoystickButton(this.driverXbox,
  // 1);

  //private final JoystickButton targetingMode = new JoystickButton(this.driverXbox, 3);

  private final POVButton ampScore = new POVButton(this.driverXbox, 0);

  private final JoystickButton ampScore1 = new JoystickButton(this.driverXbox, 10);

  private final POVButton ampScoreRetract = new POVButton(this.driverXbox, 180);
  private final POVButton climbprep = new POVButton(this.driverXbox, 90);
  private final POVButton Unicorn = new POVButton(this.driverXbox, 270);

  // private final POVButton GroundPickupExtension = new
  // POVButton(this.driverXbox, 90);
  // private final POVButton HomeExtension = new POVButton(this.driverXbox, 90);
  // private final POVButton HomePivot = new POVButton(this.driverXbox, 270);
  // private final POVButton GroundPickupPivot = new POVButton(this.driverXbox,
  // 270);

  private final JoystickButton speakerLeft = new JoystickButton(this.buttonBox, 3);
  private final JoystickButton speakerMiddle = new JoystickButton(this.buttonBox, 2);
  private final JoystickButton speakerRight = new JoystickButton(this.buttonBox, 1);
  private final JoystickButton frontLeftField = new JoystickButton(this.buttonBox, 6);
  private final JoystickButton frontMiddleField = new JoystickButton(this.buttonBox, 5);
  private final JoystickButton frontRightField = new JoystickButton(this.buttonBox, 4);
  private final JoystickButton midLeftField = new JoystickButton(this.buttonBox, 9);
  private final JoystickButton midMiddleField = new JoystickButton(this.buttonBox, 8);
  private final JoystickButton midRightField = new JoystickButton(this.buttonBox, 7);

  private final JoystickButton sourceLeft = new JoystickButton(this.buttonBox2, 1);
  private final JoystickButton backField = new JoystickButton(this.buttonBox2, 2);
  private final JoystickButton sourceRight = new JoystickButton(this.buttonBox2, 3);
  private final JoystickButton StageL = new JoystickButton(this.buttonBox2, 4);
  private final JoystickButton StageM = new JoystickButton(this.buttonBox2, 5);
  private final JoystickButton StageR = new JoystickButton(this.buttonBox2, 6);
  //private final JoystickButton ShootStage1 = new JoystickButton(this.buttonBox2, 7);
  //private final JoystickButton ShootStage2 = new JoystickButton(this.buttonBox2, 8);
  //private final JoystickButton ShootStage3 = new JoystickButton(this.buttonBox2, 9);
  private final JoystickButton amp = new JoystickButton(this.buttonBox2, 10);

  private final JoystickButton setIsAiming = new JoystickButton(this.driverXbox, 4);

  private final JoystickButton climb1f = new JoystickButton(this.alternate, 8);
  private final JoystickButton climb2f = new JoystickButton(this.alternate, 7);

  private final JoystickButton climb1r = new JoystickButton(this.alternate, 10);
  private final JoystickButton climb2r = new JoystickButton(this.alternate, 9);


    LED led = new LED();
    Shooter shooter = new Shooter(led);
    Pivot pivot = new Pivot();
    Extender extender = new Extender(led);
    Climber climber = new Climber();
    Aiming aiming = new Aiming();
    UnicornHorn unicornHorn = new UnicornHorn();

     private final SendableChooser<Command> autoChooser;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    NamedCommands.registerCommand("ampScore", CommandFactory.ampScoreCommand(extender, shooter, pivot));
    NamedCommands.registerCommand("shoot", CommandFactory.shootCommand(extender, shooter, pivot, aiming));
    NamedCommands.registerCommand("harvesterOut", CommandFactory.harvestCommand(extender, shooter, pivot));
    
    NamedCommands.registerCommand("harvesterIn", CommandFactory.retractHarvestCommandAuto(extender, shooter, pivot));
    NamedCommands.registerCommand("harvesterInNorm", CommandFactory.retractHarvestCommand(extender, shooter, pivot));


    NamedCommands.registerCommand("spoolShooter", CommandFactory.spoolShooterCommand(shooter, aiming));
    NamedCommands.registerCommand("shootQuick", CommandFactory.shootQuickCommand(shooter, aiming));

    drivebase.setupPathPlanner();

    


    autoChooser = AutoBuilder.buildAutoChooser("Auto 2-1-3");

    SmartDashboard.putData("Auto Chooser", autoChooser);

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






//    new JoystickButton(driverXbox, 4).onTrue((new InstantCommand(drivebase::zeroGyro)));
    
      // put on joystick as failsafe
    
    
    
    
    
    // new JoystickButton(driverXbox, 3).onTrue(new
    // InstantCommand(drivebase::addFakeVisionReading));

    // new JoystickButton(driverXbox,
    // 2).whileTrue(
    // Commands.deferredProxy(() -> drivebase.driveToPose(
    // new Pose2d(new Translation2d(4, 4),
    // Rotation2d.fromDegrees(0))).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)));


    // this.harvestReverse.onTrue(new InstantCommand(() ->
    // this.shooter.harvestReverse(), new Subsystem[0]));
    //this.shoot.onTrue(new InstantCommand(() -> this.shooter.shoot(), new Subsystem[0]));
    // this.indexStop.onFalse(new InstantCommand(() -> this.shooter.indexStop(), new
    // Subsystem[0]));
    
    //this.targetingMode.onTrue(new InstantCommand(() -> Vision.targetingOn(), new Subsystem[0]));
    //this.targetingMode.onFalse(new InstantCommand(() -> Vision.targetingOff(), new Subsystem[0]));

    
    this.amp.whileTrue(drivebase.driveToPath(1));//.andThen((CommandFactory.ampScoreCommand(extender, shooter, pivot))));
    this.speakerMiddle.whileTrue(drivebase.driveToPath(2).andThen((CommandFactory.shootCommand(extender, shooter, pivot, aiming))));
    this.speakerLeft.whileTrue(drivebase.driveToPath(3).andThen((CommandFactory.shootCommand(extender, shooter, pivot, aiming))));
    this.speakerRight.whileTrue(drivebase.driveToPath(4).andThen((CommandFactory.shootCommand(extender, shooter, pivot, aiming))));
    
    this.sourceLeft.whileTrue(drivebase.driveToPath(5).andThen((CommandFactory.harvestCommand(extender, shooter, pivot))));
    this.sourceRight.whileTrue(drivebase.driveToPath(6).andThen((CommandFactory.harvestCommand(extender, shooter, pivot))));
    
    this.StageL.whileTrue(drivebase.driveToPath(7));
    this.StageM.whileTrue(drivebase.driveToPath(8));
    this.StageR.whileTrue(drivebase.driveToPath(9));
    
    this.frontMiddleField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(1)));
    this.frontRightField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(2)));
    this.frontLeftField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(3)));
    this.midLeftField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(4)));
    this.midMiddleField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(5)));
    this.midRightField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(6)));
    this.backField.whileTrue(drivebase.driveToPose(ButtonMapping.buttonToPose(7)));
    
    this.setIsAiming.onTrue(new InstantCommand(() -> Aiming.setIsAiming()));
    this.setIsAiming.onFalse(new InstantCommand(() -> Aiming.setIsNotAiming()));

    this.extender.setDefaultCommand(
        new InstantCommand(() -> this.extender.extendAmount((float) -this.alternate.getRawAxis(1)), extender));
    this.pivot.setDefaultCommand(
        new InstantCommand(() -> this.pivot.moveAmount((float) this.alternate.getRawAxis(2)), pivot));
    

    this.climb1f.onTrue(new InstantCommand(() -> climber.setDesiredSpeed1()));

    this.climb1f.onFalse(new InstantCommand(() -> climber.setDesiredSpeedZero1()));

    this.climb2f.onTrue(new InstantCommand(() -> climber.setDesiredSpeed2()));

    this.climb2f.onFalse(new InstantCommand(() -> climber.setDesiredSpeedZero2()));


    this.climb1r.onTrue(new InstantCommand(() -> climber.setDesiredSpeedRev1()));

    this.climb1r.onFalse(new InstantCommand(() -> climber.setDesiredSpeedZero1()));

    this.climb2r.onTrue(new InstantCommand(() -> climber.setDesiredSpeedRev2()));

    this.climb2r.onFalse(new InstantCommand(() -> climber.setDesiredSpeedZero2()));



    this.harvest.onTrue(CommandFactory.harvestCommand(extender, shooter, pivot).alongWith(new InstantCommand(()->drivebase.setIsNotFieldOriented())).alongWith(new InstantCommand(() -> Vision.targetingOn())));
    this.harvest.onFalse(CommandFactory.retractHarvestCommand(extender, shooter, pivot).alongWith(new InstantCommand(()->drivebase.setIsFieldOriented())).alongWith(new InstantCommand(() -> Vision.targetingOff())));
    
    
   // this.harvest.onTrue((new InstantCommand(() -> Vision.targetingOn())));
    //this.harvest.onFalse((new InstantCommand(() -> Vision.targetingOff())));


    this.ampScore.onTrue(CommandFactory.ampScoreCommand(extender, shooter, pivot));
    this.ampScore1.onTrue(CommandFactory.ampScoreCommand(extender, shooter, pivot));

    this.ampScoreRetract.onTrue(CommandFactory.ampScoreRetractCommand(extender, shooter, pivot));

    this.shoot.onTrue(CommandFactory.shootCommand(extender, shooter, pivot, aiming));


    this.climbprep.onTrue(CommandFactory.climbCommand(extender, pivot));


    this.harvestManual.onTrue(new InstantCommand(() -> this.shooter.forward()));
    this.harvestManual.onFalse(new InstantCommand(() -> this.shooter.harvestStop()));

    

    this.harvestManualReverse.onTrue(new InstantCommand(() -> this.shooter.harvestReverse()));
    this.harvestManualReverse.onFalse(new InstantCommand(() -> this.shooter.harvestStop()));
    
    this.startShooter.onTrue(new InstantCommand(() -> this.shooter.startShooter()));
    this.startShooter.onFalse(new InstantCommand(() -> this.shooter.stopShooter()));
    
    this.Unicorn.onTrue(new InstantCommand(() -> this.unicornHorn.unicornExtend()));
    this.Unicorn.onFalse(new InstantCommand(() -> this.unicornHorn.unicornRetract()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command cmd =  autoChooser.getSelected();
    cmd.addRequirements(extender,pivot,shooter);
    
    return cmd;
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}