package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="Tele_op", group="1) Main OpModes")
public class Tele_op extends LinearOpMode {

    // Gate Timer for shooting
    private double shooterStartTime = 0;
    private final double GATE_OPEN_POS = .5;    // adjust
    private final double GATE_CLOSED_POS = 0.1;  // adjust
    // Drive motors
    private DcMotor fl, bl, fr, br;

    // Intake motors
    private DcMotor leftIntake, rightIntake, leftShootMotor, rightShootMotor;

    // Turret + kicker
    private CRServo leftTurretServo, rightTurretServo, kickerServo;

    private Servo gateServo;

    // Constants
    private final double MAX_POWER = 0.8;
    private final double COUNTS_PER_REV = 1024.0;

    // Variables
    private double forward, turn, strafe;
    private double lastAngle = 0;

    GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;


    @Override
    public void runOpMode() {

        // ---------------------
        // Hardware mapping
        // ---------------------
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        leftShootMotor = hardwareMap.get(DcMotor.class, "leftShootMotor");
        rightShootMotor = hardwareMap.get(DcMotor.class, "rightShootMotor");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");
        kickerServo = hardwareMap.get(CRServo.class, "kickerServo");

        gateServo = hardwareMap.get(Servo.class, "gateServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, 171.4, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Heading Scalar", pinpoint.getYawScalar());
        telemetry.update();

        // ---------------------
        // Motor directions
        // ---------------------
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // Reset intake encoder (used for angle calc)
        leftIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        gateServo.setPosition(GATE_CLOSED_POS);

        waitForStart();

        // ---------------------
        // Main loop
        // ---------------------
        double frequency = 0;
        while (opModeIsActive()) {

            // ---------------------
            // Mecanum drive
            // ---------------------
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            pinpoint.update();

            fl.setPower(forward + strafe + turn);
            bl.setPower(forward - strafe + turn);
            fr.setPower(forward - strafe - turn);
            br.setPower(forward + strafe - turn);

            // ---------------------
            // Intake (triggers)
            // ---------------------
            double intakePower = gamepad1.right_trigger * MAX_POWER;
            double outtakePower = gamepad1.left_trigger * MAX_POWER;
            double finalPower = intakePower - outtakePower;

            leftIntake.setPower(finalPower);
            rightIntake.setPower(finalPower);

            // ---------------------
            // Turret rotation (bumpers)
            // ---------------------

            if (gamepad1.a) {
                pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b) {
                pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
            }
            double newTime = getRuntime();
            double loopTime = newTime - oldTime;
            frequency = 1 / loopTime;
            oldTime = newTime;


            // D-pad DOWN → stop & close gate (toggle OFF)
// Shooter ON
            if (gamepad1.dpad_up) {
                leftShootMotor.setPower(0.7);
                rightShootMotor.setPower(-0.7);
                gateServo.setPosition(GATE_OPEN_POS);
            }

// Shooter OFF
            if (gamepad1.dpad_down) {
                gateServo.setPosition(GATE_CLOSED_POS);
                leftShootMotor.setPower(0);
                rightShootMotor.setPower(0);
            }

            if (gamepad1.dpad_left) {
                gateServo.setPosition(GATE_CLOSED_POS);
                leftShootMotor.setPower(0);
                rightShootMotor.setPower(0);
            }
                /*if (gamepad1.right_bumper) {
                    leftTurretServo.setPower(1.0);
                    rightTurretServo.setPower(1.0);
                } else if (gamepad1.left_bumper) {
                    leftTurretServo.setPower(-1.0);
                    rightTurretServo.setPower(-1.0);
                } else {
                    leftTurretServo.setPower(0);
                    rightTurretServo.setPower(0);
                }
                 */
                    if (gamepad1.x) {
                        kickerServo.setPower(1.0);
                        leftIntake.setPower(1.0);
                        rightIntake.setPower(1.0);
                    } else {
                        kickerServo.setPower(0.0);
                    }

                    // ---------------------
                    // Turret angle telemetry (from intake encoder)
                    //---------------------
                    int ticks = leftIntake.getCurrentPosition();
                    lastAngle = ((double) ticks / COUNTS_PER_REV) * 360.0 / 4 / 4;


                    //gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.

                    Pose2D pos = pinpoint.getPosition();
                    String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Position", data);


                    //gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.

                    String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH), pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
                    telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
            */
                    telemetry.addData("Status", pinpoint.getDeviceStatus());

                    telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                    telemetry.addData("Gate position:", gateServo.getPosition());
                    telemetry.update()
                    ;


                    sleep(20);
                }
            }
        }
