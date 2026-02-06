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
@Autonomous(name = "Blue12Ball", group = "Autonomous")
public class Blue12Ball extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        // ------------- positions used during auto -----------------
        Pose2d startPose = new Pose2d(39.16, 56.77, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(19.0, 13, Math.toRadians(45));
        Pose2d collectRow1 = new Pose2d(10.4, 11.0, Math.toRadians(90));
        Pose2d finishRow1 = new Pose2d(10.4, 41.0, Math.toRadians(90));
        Pose2d collectRow2 = new Pose2d(-6.5, 6.0, Math.toRadians(90));
        Pose2d finishRow2 = new Pose2d(-6.5, 40.0, Math.toRadians(90));
        Pose2d openDaGate = new Pose2d(10, 60, Math.toRadians(180));
        Pose2d collectRow3 = new Pose2d(-31,20,Math.toRadians(90));
        Pose2d finishRow3 = new Pose2d(-31,42, Math.toRadians(90));
        Pose2d leavePos = new Pose2d(5, 25, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FlyPID flywheel = new FlyPID(hardwareMap);
        Intake intake = new Intake(hardwareMap);

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
                        new SleepAction(0.0), // not needed right now

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

                        // Step 4: Stop intake at finishRow1

                        // Move to shootPose
                        drive.actionBuilder(finishRow1)
                                .setReversed(true)
                                .strafeToSplineHeading(
                                        shootPose.position,
                                        shootPose.heading)
                                .build(),

                        // Start shooting sequence
                        intake.stopIntake(),
                        new SleepAction(0.2),

                        intake.ReverseIntaking(),
                        new SleepAction(0.0),

                        flywheel.spinUp(),
                        new SleepAction(0.8),

                        intake.FastIntaking(),
                        new SleepAction(.8),

                        flywheel.idle(),

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
                                .build(),
                        intake.stopIntake(),

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
                        new SleepAction(0.0),

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
                                .build()


                        )
                        );

    }
}