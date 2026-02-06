/*package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagLimelightTest;

@Autonomous(name = "LimelightApril")
public class LimelightApril extends LinearOpMode {

    private AprilTagLimelightTest limelightMech;

    @Override
    public void runOpMode() {

        // --- Initialize your mechanism ---
        limelightMech = new AprilTagLimelightTest();
        limelightMech.init(); // calls your mechanism's init

        telemetry.addLine("Mechanism initialized, waiting for start...");
        telemetry.update();

        waitForStart();

        // --- Start Limelight ---
        limelightMech.start();

        // --- Loop while opMode is active ---
        while (opModeIsActive()) {
            limelightMech.loop(); // your mechanism updates telemetry

            telemetry.update(); // make sure telemetry from mechanism shows up
        }
    }
}

 */
