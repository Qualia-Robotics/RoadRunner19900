package org.firstinspires.ftc.teamcode.Teleop;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
@TeleOp(name="Teleop", group="1) Main OpModes")
public class Tele_op extends LinearOpMode {

    public DcMotor fl, bl, fr, br,intake,shootm,transfer;
    public double forward, turn, strafe1,shootp,power,transferpos;




    @Override
    public void runOpMode() {


        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        intake= hardwareMap.get(DcMotor.class,"intake");
        shootm= hardwareMap.get(DcMotor.class,"shoot");
        transfer= hardwareMap.get(DcMotor.class,"transfer");
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        power=0;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // INIT LOOP -------------------------------------------------------------------------------

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Shoot Power", shootm.getPower());
            telemetry.update();
            forward = -gamepad1.left_stick_y;

            turn = -gamepad1.right_stick_x;
            strafe1 = gamepad1.left_stick_x;
            fl.setPower((-forward) + (turn * .8) - strafe1);
            bl.setPower((-forward) + (turn * .8) + strafe1);
            fr.setPower((-forward) - (turn * .8) + strafe1);
            br.setPower((-forward) - (turn * .8) - strafe1);
            shootm.setPower(shootp);
            transfer.setPower(transferpos);
            if (gamepad1.triangle) {
                intake.setPower(-0.8);
            }
            if (gamepad1.circle) {
                intake.setPower(0);
                transferpos=0;
            }
            if (gamepad1.square){
                intake.setPower(0.5);
            }
            if(gamepad1.dpad_left){
                shootp=(-.05);

            }
            if(gamepad1.dpad_up){
                shootp=(.54);
            }
            if(gamepad1.dpad_right){
                shootp = (-1);
            }
            if(gamepad1.dpad_down) {
                shootp = (0);
            }
            if(gamepad1.left_bumper){
                transferpos=1;
            }
            if(gamepad1.right_bumper){
                transferpos=-0.36;
            }


        }}}