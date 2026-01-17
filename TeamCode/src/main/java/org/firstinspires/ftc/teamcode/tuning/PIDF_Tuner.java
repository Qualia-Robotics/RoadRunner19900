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

    }
}