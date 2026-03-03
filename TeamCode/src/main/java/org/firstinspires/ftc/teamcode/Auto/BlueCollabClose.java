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
import org.firstinspires.ftc.teamcode.mechanisms.Intake;


@Config
@Autonomous(name = "BlueCollabClose", group = "Autonomous")
public class BlueCollabClose extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d startPose = new Pose2d(39.16, 56.77, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(19.0, 13, Math.toRadians(49));
        Pose2d shootPose2 = new Pose2d(19.0, 13, Math.toRadians(51));
        Pose2d collectRow1 = new Pose2d(13.4, 11.0, Math.toRadians(90));
        Pose2d finishRow1 = new Pose2d(13.4, 38.0, Math.toRadians(90));
        Pose2d collectRow2 = new Pose2d(-7.1, 8.0, Math.toRadians(90));
        Pose2d finishRow2 = new Pose2d(-7.1, 43.0, Math.toRadians(90));
        Pose2d openDaGate = new Pose2d(10, 57, Math.toRadians(180));
        Pose2d openDaGate2 = new Pose2d(0, 57, Math.toRadians(0));
        Pose2d alignRamp1 = new Pose2d(-7, 42.5, Math.toRadians(70));
        Pose2d alignRamp2 = new Pose2d(-7, 56.5, Math.toRadians(60));
        Pose2d alignRamp3 = new Pose2d(-20.5, 54.5, Math.toRadians(50));
        Pose2d leavePos = new Pose2d(5, 25, Math.toRadians(90));


        //Pose2d collectRow3 = new Pose2d(-24,-20,Math.toRadians(-90));
        //Pose2d finishRow3 = new Pose2d(-24,-50, Math.toRadians(-90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FlyPID flywheel = new FlyPID(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        double strafeScale = 0.5; // 50% speed for strafes


        // Shoot --- Start
        Actions.runBlocking(
                new SequentialAction(

                        // MOVE INTO SHOOT POSITION
                        flywheel.spinUp(),

                        drive.actionBuilder(startPose)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading
                                )
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        new SleepAction(0.1),

                        intake.FastIntaking(),

                        new SleepAction(.6),

                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        collectRow2.position,
                                        collectRow2.heading)

                                .build(),
                        drive.actionBuilder(collectRow2)
                                .strafeToLinearHeading(
                                        finishRow2.position,
                                        finishRow2.heading)

                                .build(),
                        intake.stopIntake(),
                        // OPEN DA GATE
                        drive.actionBuilder(finishRow2)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        openDaGate.position,
                                        openDaGate.heading)
                                .build(),

                        // MOVE INTO SHOOT POSITION
                        drive.actionBuilder(collectRow2)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading)
                                .build(),

                        // ===== SHOOT SEQUENCE =====
                        intake.stopIntake(),
                        new SleepAction(0.2),

                        intake.ReverseIntaking(),
                        new SleepAction(0.0),

                        flywheel.spinUp(),
                        new SleepAction(0.8),

                        intake.FastIntaking(),
                        new SleepAction(1),

                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 1
                        drive.actionBuilder(collectRow1)
                                .strafeToLinearHeading(
                                        finishRow1.position,
                                        finishRow1.heading)
                                .build(),
                        intake.stopIntake(),

                        drive.actionBuilder(finishRow1)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        openDaGate2.position,
                                        openDaGate2.heading)
                                .build(),

                        // Step 4: Stop intake at finishRow1

                        // Move to shootPose
                        drive.actionBuilder(openDaGate2)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading)
                                .build(),

                        // Start shooting sequence

                        intake.ReverseIntaking(),
                        new SleepAction(0.0),

                        flywheel.spinUp(),
                        new SleepAction(0.8),

                        intake.FastIntaking(),
                        new SleepAction(.8),

                        flywheel.idle(),

                        // MOVE TO COLLECT ROW 2 with different heading
                        drive.actionBuilder(shootPose)
                                .strafeToLinearHeading(
                                        alignRamp1.position,
                                        alignRamp1.heading)
                                .build(),
                        drive.actionBuilder(alignRamp2)
                                .strafeToLinearHeading(
                                        alignRamp3.position,
                                        alignRamp3.heading)
                                .build(),
                        intake.FastIntaking(),
                        new SleepAction(1.4),
                        intake.stopIntake(),
                        drive.actionBuilder(collectRow2)
                                .strafeToLinearHeading(
                                        shootPose2.position,
                                        shootPose2.heading)
                                .build(),
                        //------SHOOT CYCLE------//
                        intake.stopIntake(),
                        new SleepAction(0.2),

                        intake.ReverseIntaking(),
                        new SleepAction(0.0),

                        flywheel.spinUp(),
                        new SleepAction(0.8),

                        intake.FastIntaking(),
                        new SleepAction(.8),

                        flywheel.stop(),

                        drive.actionBuilder(shootPose2)
                                .strafeToLinearHeading(
                                        leavePos.position,
                                        leavePos.heading
                                )
                                .build()
                        /*
                        // Move to shootPose
                        drive.actionBuilder(finishRow3)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading)
                                .build(),

                        // Start shooting sequence
                        intake.stopIntake(),
                        new SleepAction(0.2),

                        intake.ReverseIntaking(),
                        new SleepAction(0.15),

                        flywheel.spinUp(),
                        new SleepAction(0.8),

                        intake.FastIntaking(),
                        new SleepAction(.8),

                        intake.stopIntake(),
                        flywheel.stop(),
                        new SleepAction(0.05),

                        drive.actionBuilder(shootPose)
                                .setReversed(true)
                                .strafeToSplineHeading(leavePos.position, leavePos.heading)
                                .build()*/


                )
        );

    }
}