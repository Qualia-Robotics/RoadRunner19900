package org.firstinspires.ftc.teamcode.Auto;

// RR-specific imports

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shoot;


@Config
@Autonomous(name = "PassThroughAuto", group = "Autonomous")
public class PassThroughAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d startPose = new Pose2d(40.66, -56.77, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(27.0, -19.3, Math.toRadians(-45));
        Pose2d collectRow1 = new Pose2d(21.4, -10.0, Math.toRadians(-90));
        Pose2d finishRow1 = new Pose2d(21.4, -47.0, Math.toRadians(-90));
        Pose2d collectRow2 = new Pose2d(-2.4, -5.0, Math.toRadians(-90));
        Pose2d finishRow2 = new Pose2d(-2.4, -50.0, Math.toRadians(-90));
        Pose2d collectRow3 = new Pose2d(-26,-20,Math.toRadians(-90));
        Pose2d finishRow3 = new Pose2d(-26,-50, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Shoot Shoot = new Shoot(hardwareMap);
        Intake Intake = new Intake(hardwareMap);

        double strafeScale = 0.5; // 50% speed for strafes


        // Shoot --- Start
        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        Shoot.shootScore(),

                        drive.actionBuilder(startPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(1),

                        Intake.FastIntaking(),
                        new SleepAction(.8),

                        Shoot.stopShooting(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(collectRow1)
                                .strafeToLinearHeading(finishRow1.position, finishRow1.heading)

                                .build()

                        // Step 4: Stop intake at finishRow1
                )
        );

        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        drive.actionBuilder(finishRow1)
                                .setReversed(true)
                                .strafeToSplineHeading(shootPose.position, shootPose.heading)
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        Intake.stopIntake(),
                        new SleepAction(0.2),
                        Shoot.shootScore(),
                        new SleepAction(2.5),

                        Intake.FastIntaking(),
                        new SleepAction(1.8),

                        Shoot.stopShooting(),

                        // MOVE TO COLLECT ROW 2
                        drive.actionBuilder(collectRow2)
                                .strafeToLinearHeading(finishRow2.position, finishRow2.heading)
                                .build()

                        // Step 4: Stop intake at finishRow2

                )
        );

        //Finish Row 2 --- Shoot
        Actions.runBlocking(
                new SequentialAction(
                        // Move to shootPose
                        drive.actionBuilder(finishRow2)
                                .setReversed(true)
                                .strafeToSplineHeading(shootPose.position, shootPose.heading)
                                .build(),

                        // Start shooting sequence
                        Intake.stopIntake(),
                        new SleepAction(0.2),
                        Shoot.shootScore(),
                        new SleepAction(2.5),

                        Intake.FastIntaking(),
                        new SleepAction(.8),

                        Shoot.stopShooting(),

                        // MOVE TO COLLECT ROW 3
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        collectRow3.position,
                                        collectRow3.heading
                                )
                                .build(),

                        drive.actionBuilder(collectRow3)
                                .strafeToLinearHeading(
                                        finishRow3.position,
                                        finishRow3.heading
                                )
                                .build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        // Move to shootPose
                        drive.actionBuilder(finishRow3)
                                .setReversed(true)
                                .strafeToSplineHeading(shootPose.position, shootPose.heading)
                                .build(),

                        // Start shooting sequence
                        Intake.stopIntake(),
                        new SleepAction(0.2),
                        Shoot.shootScore(),
                        new SleepAction(2.5),

                        Intake.FastIntaking(),
                        new SleepAction(.8),

                        Shoot.stopShooting()
                )
        );

    }
}