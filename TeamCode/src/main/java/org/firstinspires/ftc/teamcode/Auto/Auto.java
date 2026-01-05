package org.firstinspires.ftc.teamcode.Auto;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shoot;



@Config
@Autonomous(name = "Auto", group = "Autonomous")
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        waitForStart();
        Pose2d currentPose = new Pose2d(40.66, -56.77, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Shoot Shoot = new Shoot(hardwareMap);
        Intake Intake = new Intake(hardwareMap);

        TrajectoryActionBuilder ShootPos = drive.actionBuilder(currentPose)

                //.splineToSplineHeading(new Pose2d(17, -9.3, Math.toRadians(-45)), Math.toRadians(-300));
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(17, -9.3), Math.toRadians(-45));
                currentPose = new Pose2d(17, -9.3, Math.toRadians(-45));

        //.waitSeconds(1);
        TrajectoryActionBuilder collectrow1 = drive.actionBuilder(currentPose)
            .strafeToLinearHeading(new Vector2d(21.4, -24), Math.toRadians(-90));
        currentPose = new Pose2d(21.4, -24, Math.toRadians(-90));

        TrajectoryActionBuilder finishrow1 = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(21.4, -48), Math.toRadians(-90));
        currentPose = new Pose2d(21.4, -48, Math.toRadians(-90));

        TrajectoryActionBuilder collectrow2 = drive.actionBuilder(currentPose)
                .setTangent(-45)
                .strafeToLinearHeading(new Vector2d(-2.6, -24), Math.toRadians(-90));
        currentPose = new Pose2d(-2.6, -24, Math.toRadians(-90));

        TrajectoryActionBuilder finishrow2 = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-2.6, -48), Math.toRadians(-90));
        currentPose = new Pose2d(-2.6, -48, Math.toRadians(-90));


        Actions.runBlocking(
                new ParallelAction(
                new SequentialAction(
                        ShootPos.build(),
                        Shoot.shootScore(), new SleepAction(1.75), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(),
                        Shoot.stopShooting(),
                        collectrow1.build(),
                        Intake.Intaking(),
                        finishrow1.build(),
                        Intake.stopIntake(),
                        //ShootPos.build(),
                        drive.actionBuilder(currentPose)
                                .setReversed(true)
                                .strafeToLinearHeading(
                                        new Vector2d(17, -9.3),
                                        Math.toRadians(-45)
                                )
                                .build(),
                        Shoot.shootScore(), new SleepAction(1.5), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.5), Intake.stopIntake(),
                        Shoot.stopShooting(),
                        collectrow2.build(),
                        Intake.Intaking(),
                        finishrow2.build(),
                        Intake.stopIntake(),
                        //ShootPos.build(),
                        drive.actionBuilder(currentPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        new Vector2d(17, -9.3),
                                        Math.toRadians(-45)
                                )
                                .build(),
                        Shoot.shootScore(), new SleepAction(2), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.2), Intake.stopIntake(), new SleepAction(.3), Intake.Intaking(), new SleepAction(.5), Intake.stopIntake(),
                        Shoot.stopShooting()

                ))
        );
    }
    }