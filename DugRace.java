package Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class DugRace extends LinearOpMode {
    DcMotorEx s,d;
    Servo directieS,directieD;

    double fata,lateral;
    double viteza=0.8,vitezaServo=0.1,vitezaMare=1;
    public void miscare(){
        boolean a,b;
        fata= Range.clip(gamepad1.left_stick_x,-viteza,viteza);
        lateral=Range.clip(gamepad1.right_stick_x,vitezaServo,-vitezaServo);
        s.setPower(fata);
        d.setPower(fata);
        a= gamepad1.a;
        b= gamepad1.b;
        if(a)
        {
            viteza=vitezaMare;
        }
        if(b)
        {
            viteza=0.8;
        }

        directieS.setPosition(0.5+lateral);
        directieD.setPosition(0.5+lateral);
        telemetry.addData("Position: ",directieS.getPosition());
        telemetry.update();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        s=hardwareMap.get(DcMotorEx.class,"stanga");
        d=hardwareMap.get(DcMotorEx.class,"dreapta");
        directieS=hardwareMap.get(Servo.class,"directies");
        directieD=hardwareMap.get(Servo.class,"directied");


        s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        d.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()&&!isStopRequested()){

            miscare();

        }
    }

}

