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
@Autonomous(name = "Red_Far", group = "Autonomous")
public class Red_Far extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d shootPose = new Pose2d(7.8, 6.7, Math.toRadians(-24));
        Pose2d intakePose = new Pose2d(35.1, -56.5, Math.toRadians(180));
        Pose2d pickUpPose = new Pose2d(13.5, -55, Math.toRadians(180));




        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        FlyPID flywheel = new FlyPID(hardwareMap);
        Intake intake = new Intake(hardwareMap);

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
                        new SleepAction(.5),
                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.4),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        intake.stopIntake(),
                        new SleepAction(.4),

                        intake.FastIntaking(),
                        new SleepAction(.15),
                        flywheel.stop(),

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

                                .build()

                        )
                        );

    }
}