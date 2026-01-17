package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

// no usages
@TeleOp
public class PIDF_Tuner extends OpMode {
    public DcMotorEx leftShootMotor,rightShootMotor;
    public double highVelocity=1500;
    public double lowVelocity=900;
    public double curTarVel = highVelocity;
    public double F=0;
    public double P=0;
    double[] stepSizes = {10, 1, 0.1, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        leftShootMotor = hardwareMap.get(DcMotorEx.class,"leftShootMotor");
        leftShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShootMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightShootMotor = hardwareMap.get(DcMotorEx.class,"rightShootMotor");
        rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop() {
        //All gamepad command to add
        //Set target Velo
        //Update Telmet
        if(gamepad1.triangleWasPressed()){
            if (curTarVel==highVelocity){
                curTarVel= lowVelocity;
            } else {curTarVel=highVelocity;}
        }
        if(gamepad1.squareWasPressed()){
            stepIndex=(stepIndex+1) % stepSizes.length;

        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -=stepSizes[stepIndex];
        }
        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -=stepSizes[stepIndex];
        }
        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        leftShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //velocity
        leftShootMotor.setVelocity(curTarVel);
        rightShootMotor.setVelocity(curTarVel);
        double curVel= leftShootMotor.getVelocity();
        double error= curTarVel-curVel;
        telemetry.addData("Target Velocity", curTarVel);
        telemetry.addData("Current Velocity", curVel);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Target Velocity", curTarVel);
        telemetry.addLine();
        telemetry.addData("P","%.4f(Dpad Up/Down)", P);
        telemetry.addData("F","%.4f(Dpad Right/Left)", F);
        telemetry.addData("Step Size","%.4f(Square)", stepSizes[stepIndex]);

    }
}