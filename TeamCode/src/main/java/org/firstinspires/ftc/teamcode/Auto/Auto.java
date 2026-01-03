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
        Pose2d initialPose = new Pose2d(40.66, -56.77, Math.toRadians(0));
        //Pose2d shootPose = new Pose2d(17, -9.3, Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Shoot Shoot = new Shoot(hardwareMap);
        Intake Intake = new Intake(hardwareMap);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                //lineToYSplineHeading(-9.3, Math.toRadians(-45))
                //.waitSeconds(2)
                //.setTangent(Math.toRadians(90))
                //.lineToX(17)
                //.lineToY(-9.3)
                //.setTangent(Math.toRadians(0))
                //.turnTo(-45);
                //.setTangent(0)
                //.splineToSplineHeading(new Pose2d(17, -9.3, Math.toRadians(-45)), Math.toRadians(-300));
                .strafeToLinearHeading(new Vector2d(17, -9.3), Math.toRadians(-45));
                //.waitSeconds(1);

                //.setTangent(0)
                //.splineToSplineHeading(new Pose2d(20.7,-22.5,-90), Math.PI/2);
                //.strafeToHea(new Vector2d(17, -9.3))//score position/angle
                //.turn(Math.toRadians(-45))
                //shoot here
                //.lineToY(-50);


        Actions.runBlocking(
                new ParallelAction(
                new SequentialAction(
                        tab1.build(),
                        Shoot.shootScore(),
                        Intake.Intaking(),
                        new SleepAction(500),
                        Intake.stopIntake()
                ))
        );
    }
    }