// package frc.robot.auto.center;

// import java.io.IOException;

// import org.json.simple.parser.ParseException;

// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.ShootCommand;
// import frc.robot.constants.Constants;
// import frc.robot.subsystems.HubAlignmentPID;
// import frc.robot.subsystems.hopper.Hopper;
// import frc.robot.subsystems.indexer.Indexer;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.Pivot;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.hood.Hood;
// import frc.robot.subsystems.HubAlignmentPID;
// import frc.robot.subsystems.autoalignhood.Shootercalculations
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// public class ShootAndDriveToNeutral extends SequentialCommandGroup {
//      public ShootAndDriveToNeutral(CommandSwerveDrivetrain swerve, Shooter shooter, Hood hood, Hopper hopper, Indexer indexer, Intake intake, Pivot pivot) {

//         Command resetPose = new InstantCommand(), neutralZoneToAlliance, CenterDriveOverBump, neutralZoneIntake;
//         Pose2d startingPose;

//         try {
//             CenterDriveOverBump = swerve.driveAlongPath(PathPlannerPath.fromPathFile("CenterDriveOverBump"));
//             if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
//                 startingPose = PathPlannerPath.fromPathFile("CenterDriveOverBump").getStartingHolonomicPose().get();
//                 resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
//             } else {
//                 startingPose = PathPlannerPath.fromPathFile("CenterDriveOverBump").flipPath().getStartingHolonomicPose().get();
//                 resetPose = new InstantCommand(() -> swerve.resetPose(startingPose), swerve);
//             }
//         } catch (IOException | ParseException e) {
//             e.printStackTrace();
//             CenterDriveOverBump = null; // or handle the error appropriately
//         }
//         try {
//             neutralZoneIntake = swerve.driveAlongPath(PathPlannerPath.fromPathFile("neutralZoneIntake"));
//         } catch (IOException | ParseException e) {
//             e.printStackTrace();
//             neutralZoneIntake = null; // or handle the error appropriately
//         }
//         try {
//             neutralZoneToAlliance = swerve.driveAlongPath(PathPlannerPath.fromPathFile("neutralZoneToAlliance"));
//         } catch (IOException | ParseException e) {
//             e.printStackTrace();
//             neutralZoneToAlliance = null; // or handle the error appropriately
//         }


//         addRequirements(swerve);

//         addCommands(
//             resetPose,
//             new ParallelDeadlineGroup(
//                 new ShootCommand(swerve, HubAlignmentPID, shooter, hood, indexer, hopper, shootercalc )
//                 startToL3,
//                 new IntakeCoralFull(coralShooter)
//             ),
//             new LowAlgaeGrabNoDriver(swerve, elevator, coralShooter, algaePivot, algaeClaw),
//             l3ToProcessor,
//             new ProcessAlgae(elevator, algaePivot, algaeClaw),
//             processorToHighAlgae,
//             new ParallelCommandGroup(
//                 new CloseDriveToClosestReefGoodOffset(swerve),
//                 new ElevatorPreset(elevator, Constants.ElevatorConstants.HIGH_ALGAE_ENCODER_TICKS)
//             ),
//             new AlgaePivotPreset(algaePivot, Constants.AlgaeClawConstants.PIVOT_OUT_TICKS),
//             new ParallelCommandGroup(
//                 new AutoLineUpReefUniversal(swerve, 0),
//                 new GrabAlgae(algaeClaw)
//             ),
//             new ParallelDeadlineGroup(
//                 new CloseDriveToClosestAlgaeOffset(swerve),
//                 new GrabAlgaeTime(algaeClaw, 0.5)
//             ),
//             new RestMode(elevator, algaePivot, algaeClaw),
//             highAlgaeToProcessor,
//             new ProcessAlgae(elevator, algaePivot, algaeClaw)
//         );

//     }
// }
