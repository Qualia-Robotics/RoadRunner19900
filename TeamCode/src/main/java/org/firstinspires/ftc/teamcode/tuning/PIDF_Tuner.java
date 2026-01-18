package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDF_Tuner extends OpMode {
    public DcMotorEx leftShootMotor, rightShootMotor;
    public double highVelocity = 1250;
    public double lowVelocity = 0;
    public double curTarVel = highVelocity;
    public double F = 0;
    public double P = 0;
    double[] stepSizes = {10, 5 , 1, 0.5, 0.1, 0.05, 0.01 ,0.005, 0.001, 0.0005 , 0.0001};

    int stepIndex = 1;

    // Timer to measure spin-up
    private ElapsedTime spinupTimer = new ElapsedTime();
    private boolean timing = false; // true while flywheel is accelerating
    private double spinupTime = 0;  // last recorded spinup time

    @Override
    public void init() {
        leftShootMotor = hardwareMap.get(DcMotorEx.class,"leftShootMotor");
        leftShootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShootMotor.setDirection(DcMotorEx.Direction.FORWARD);

        rightShootMotor = hardwareMap.get(DcMotorEx.class,"rightShootMotor");
        rightShootMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftShootMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // Switch target velocity
        if(gamepad1.triangleWasPressed()){
            if (curTarVel == highVelocity){
                curTarVel = lowVelocity;
            } else {
                curTarVel = highVelocity;

                // Start timer when switching to high velocity
                spinupTimer.reset();
                timing = true;
            }
        }

        // Step size adjustments
        if(gamepad1.squareWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        // Update PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftShootMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set flywheel velocity
        leftShootMotor.setVelocity(curTarVel);
        rightShootMotor.setPower(leftShootMotor.getPower());

        // Measure current velocity
        double curVel = leftShootMotor.getVelocity();
        double error = curTarVel - curVel;

        // Check spin-up completion (97% of target)
        if (timing && curVel >= curTarVel * 0.97) {
            spinupTime = spinupTimer.seconds();
            timing = false; // stop timing
        }

        // Telemetry
        telemetry.addData("Target Velocity", curTarVel);
        telemetry.addData("Current Velocity", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine();
        telemetry.addData("P","%.4f (Dpad Up/Down)", P);
        telemetry.addData("F","%.4f (Dpad Right/Left)", F);
        telemetry.addData("Step Size","%.4f (Square)", stepSizes[stepIndex]);
        telemetry.addLine();

        // Spin-up timer telemetry
        if (curTarVel == highVelocity) {
            if (timing) {
                telemetry.addData("Spin-up Timer", "%.3f s (spinning up)", spinupTimer.seconds());
            } else {
                telemetry.addData("Spin-up Time", "%.3f s", spinupTime);
            }
        }

        telemetry.update();
    }
}
