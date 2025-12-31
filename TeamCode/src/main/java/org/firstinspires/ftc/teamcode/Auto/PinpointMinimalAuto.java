package org.firstinspires.ftc.teamcode.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import java.util.Locale;
@Autonomous(name = "Pinpoint Minimal Auto", group = "Test")
public class PinpointMinimalAuto extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        // Initialize motors
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        br = hardwareMap.get(DcMotor.class, "rightBack");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Initialize pinpoint and dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        Pose2D startPos = pinpoint.getPosition(); // record start

        pinpoint.resetPosAndIMU();

        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();


            double deltaX = pos.getX(DistanceUnit.INCH) - startPos.getX(DistanceUnit.INCH);
            double deltaY = pos.getY(DistanceUnit.INCH) - startPos.getY(DistanceUnit.INCH);

            // motor driving logic
            fl.setPower(.3);
            bl.setPower(.3);
            fr.setPower(.3);
            br.setPower(.3);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x", pos.getX(DistanceUnit.INCH));
            packet.put("y", pos.getY(DistanceUnit.INCH));
            packet.put("heading", pos.getHeading(AngleUnit.DEGREES));

            // Draw robot position relative to start (origin at 0,0)
            packet.fieldOverlay()
                    .setStrokeWidth(2)
                    .setStroke("red")
                    .fillCircle((float) deltaX, (float) deltaY, 3);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X", pos.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pos.getY(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}