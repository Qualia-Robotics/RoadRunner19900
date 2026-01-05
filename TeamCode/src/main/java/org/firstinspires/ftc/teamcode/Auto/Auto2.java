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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shoot;


@Config
@Autonomous(name = "Auto2", group = "Autonomous")
public class Auto2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d startPose = new Pose2d(40.66, -56.77, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(17.0, -9.3, Math.toRadians(-45));
        Pose2d collectRow1 = new Pose2d(21.4, -24.0, Math.toRadians(-90));
        Pose2d finishRow1 = new Pose2d(21.4, -48.0, Math.toRadians(-90));
        Pose2d collectRow2 = new Pose2d(-2.6, -24.0, Math.toRadians(-90));
        Pose2d finishRow2 = new Pose2d(-2.6, -48.0, Math.toRadians(-90));
        Pose2d collectRow3 = new Pose2d(-26.6,-24,Math.toRadians(-90));
        Pose2d finishRow3 = new Pose2d(-26.6,-48, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shoot Shoot = new Shoot(hardwareMap);
        Intake Intake = new Intake(hardwareMap);

        // Shoot --- Start
        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        drive.actionBuilder(startPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        Shoot.shootScore(),
                        new SleepAction(1.75),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),

                        Shoot.stopShooting(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        collectRow1.position,
                                        collectRow1.heading
                                )
                                .build(),

                        // Step 2: Turn on intake at collectRow1
                        Intake.Intaking(),

                        // Step 3: Move to finishRow1 while intake is running
                        drive.actionBuilder(collectRow1)
                                .strafeToLinearHeading(
                                        finishRow1.position,
                                        finishRow1.heading
                                )
                                .build(),

                        // Step 4: Stop intake at finishRow1
                        Intake.stopIntake()
                )
        );

        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        drive.actionBuilder(collectRow1)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        Shoot.shootScore(),
                        new SleepAction(1.75),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),

                        Shoot.stopShooting(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        collectRow2.position,
                                        collectRow2.heading
                                )
                                .build(),

                        // Step 2: Turn on intake at collectRow1
                        Intake.Intaking(),

                        // Step 3: Move to finishRow1 while intake is running
                        drive.actionBuilder(collectRow1)
                                .strafeToLinearHeading(
                                        finishRow2.position,
                                        finishRow2.heading
                                )
                                .build(),

                        // Step 4: Stop intake at finishRow1
                        Intake.stopIntake()
                )
        );

        //Finish Row 2 --- Shoot
        Actions.runBlocking(
                new SequentialAction(
                        // Move to shootPose
                        drive.actionBuilder(finishRow2)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // Start shooting sequence
                        Shoot.shootScore(),
                        new SleepAction(1.75),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),
                        new SleepAction(.3),

                        Intake.Intaking(),
                        new SleepAction(.2),
                        Intake.stopIntake(),

                        Shoot.stopShooting()
                )
        );
    }
}