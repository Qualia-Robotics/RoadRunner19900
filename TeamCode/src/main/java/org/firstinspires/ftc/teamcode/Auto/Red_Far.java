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
import org.firstinspires.ftc.teamcode.mechanisms.FlyPID;
import org.firstinspires.ftc.teamcode.mechanisms.Gate;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;


@Config
@Autonomous(name = "Red_Far", group = "Autonomous")
public class Red_Far extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d leavePose = new Pose2d(20, 0, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(7.8, 6.7, Math.toRadians(-24));
        Pose2d shootPose2 = new Pose2d(7.8, 6.7, Math.toRadians(-27));
        Pose2d intakePose = new Pose2d(35.1, -56.5, Math.toRadians(180));
        Pose2d pickUpPose = new Pose2d(15.5, -55, Math.toRadians(180));
        Pose2d backUpPose = new Pose2d(28.5, -55, Math.toRadians(180));

        Pose2d passThroughPose = new Pose2d(16.5, -50, Math.toRadians(180));
        Pose2d intakePose2 = new Pose2d(10, -22, Math.toRadians(-90));
        Pose2d pickUpPose2 = new Pose2d(51.5, -47.5, Math.toRadians(-5));
        Pose2d shootPassThroughPose = new Pose2d(2.6, -36.2, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FlyPID flywheel = new FlyPID(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Gate gate = new Gate(hardwareMap);


        // Shoot --- Start
        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        flywheel.spinUpFar(),

                        drive.actionBuilder(startPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(1),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.3),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        //flywheel.idle(),
                        gate.Close(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        intakePose.position,
                                        intakePose.heading)

                                .build(),
                        drive.actionBuilder(intakePose)
                                .strafeToLinearHeading(
                                        pickUpPose.position,
                                        pickUpPose.heading)

                                .build(),

                        drive.actionBuilder(passThroughPose)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        //flywheel.spinUpFar(),

                        // ===== SHOOT SEQUENCE =====
                        //new SleepAction(1),
                        gate.Open(),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.3),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        //flywheel.idle(),
                        gate.Close(),
                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        intakePose.position,
                                        intakePose.heading)

                                .build(),
                        drive.actionBuilder(intakePose)
                                .strafeToLinearHeading(
                                        pickUpPose.position,
                                        pickUpPose.heading)

                                .build(),

                        drive.actionBuilder(passThroughPose)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        flywheel.spinUpFar(),

                        // ===== SHOOT SEQUENCE =====
                        //new SleepAction(1),
                        gate.Open(),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.3),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        //flywheel.idle(),
                        gate.Close(),
                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        intakePose.position,
                                        intakePose.heading)

                                .build(),
                        drive.actionBuilder(intakePose)
                                .strafeToLinearHeading(
                                        pickUpPose.position,
                                        pickUpPose.heading)

                                .build(),

                        drive.actionBuilder(passThroughPose)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        flywheel.spinUpFar(),


                        // ===== SHOOT SEQUENCE =====
                        //new SleepAction(1),
                        gate.Open(),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.3),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        flywheel.stop(),
                        gate.Close(),
                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        leavePose.position,
                                        leavePose.heading)

                                .build()

                )
                        );


       /* // Shoot --- Start
        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        flywheel.spinUpFar(),

                        drive.actionBuilder(startPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(.5),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        intakePose.position,
                                        intakePose.heading)

                                .build(),
                        drive.actionBuilder(intakePose)
                                .strafeToLinearHeading(
                                        pickUpPose.position,
                                        pickUpPose.heading)

                                .build(),
                        drive.actionBuilder(passThroughPose)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        flywheel.spinUpFar(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(1),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        intakePose2.position,
                                        intakePose2.heading)

                                .build(),
                        drive.actionBuilder(intakePose2)
                                .strafeToLinearHeading(
                                        pickUpPose2.position,
                                        pickUpPose2.heading)

                                .build(),
                        drive.actionBuilder(pickUpPose2)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        flywheel.spinUpFar(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(0.8),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        intakePose2.position,
                                        intakePose2.heading)

                                .build(),
                        drive.actionBuilder(intakePose2)
                                .strafeToLinearHeading(
                                        pickUpPose2.position,
                                        pickUpPose2.heading)

                                .build(),
                        drive.actionBuilder(pickUpPose2)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)

                                .build(),

                        intake.stopIntake(),
                        flywheel.spinUpFar(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(0.8),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.2),

                        intake.FastIntaking(),
                        new SleepAction(.3),
                        flywheel.stop(),


                drive.actionBuilder(shootPose2)
                        .strafeToLinearHeading(
                                leavePose.position,
                                leavePose.heading)

                        .build()
                )
                        );*/


    }
}