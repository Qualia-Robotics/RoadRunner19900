package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.Locale;

public class Shoot {

    private DcMotorEx leftShootMotor, rightShootMotor;

    public Shoot(HardwareMap hardwareMap) {
        DcMotorEx leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        DcMotorEx rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
    }
}
