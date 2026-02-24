package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDF_Tuner extends OpMode {

    public DcMotorEx leftShootMotor, rightShootMotor;

    public double highVelocity = 1550;
    public double midVelocity  = 1100;
    public double lowVelocity  = 0;

    public double curTarVel = lowVelocity;

    public double F = 0;
    public double P = 0;

    double[] stepSizes = {
            10, 5, 1, 0.5, 0.1,
            0.05, 0.01, 0.005,
            0.001, 0.0005, 0.0001
    };

    int stepIndex = 1;

    // Spin-up timing
    private ElapsedTime spinupTimer = new ElapsedTime();
    private boolean timing = false;
    private double spinupTime = 0;

    @Override
    public void init() {

        leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        leftShootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShootMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
        rightShootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        leftShootMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER, pidf
        );

        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        /*
         * =========================
         * Velocity Cycling
         * LOW → MID → HIGH → LOW
         * =========================
         */
        if (gamepad1.triangleWasPressed()) {

            double previousTarget = curTarVel;

            if (curTarVel == lowVelocity) {
                curTarVel = midVelocity;

            } else if (curTarVel == midVelocity) {
                curTarVel = highVelocity;

            } else {
                curTarVel = lowVelocity;
            }

            // Start timer only if accelerating upward
            if (curTarVel > previousTarget) {
                spinupTimer.reset();
                timing = true;
            }
        }

        /*
         * =========================
         * Step Size Adjustment
         * =========================
         */
        if (gamepad1.squareWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        /*
         * =========================
         * Apply PIDF
         * =========================
         */
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        leftShootMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER, pidf
        );

        /*
         * =========================
         * Set Velocity
         * =========================
         */
        leftShootMotor.setVelocity(curTarVel);
        rightShootMotor.setPower(leftShootMotor.getPower());

        double curVel = leftShootMotor.getVelocity();
        double error = curTarVel - curVel;

        /*
         * =========================
         * Spin-up Detection (97%)
         * =========================
         */
        if (timing && curTarVel > 0 &&
                curVel >= curTarVel * 0.97) {

            spinupTime = spinupTimer.seconds();
            timing = false;
        }

        /*
         * =========================
         * Telemetry
         * =========================
         */
        telemetry.addData("Target Velocity", curTarVel);
        telemetry.addData("Current Velocity", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine();

        telemetry.addData("P", "%.4f (Dpad Up/Down)", P);
        telemetry.addData("F", "%.4f (Dpad Right/Left)", F);
        telemetry.addData("Step Size",
                "%.4f (Square)", stepSizes[stepIndex]);
        telemetry.addLine();

        if (timing) {
            telemetry.addData("Spin-up Timer",
                    "%.3f s (spinning up)",
                    spinupTimer.seconds());
        } else if (spinupTime > 0) {
            telemetry.addData("Last Spin-up Time",
                    "%.3f s", spinupTime);
        }

        telemetry.update();
    }
}