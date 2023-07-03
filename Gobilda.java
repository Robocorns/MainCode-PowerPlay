package Teleop;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Gobilda TeleOp", group = "A")
public class Gobilda extends LinearOpMode {
    private ElapsedTime     timer = new ElapsedTime();
    private double lastError=0;

    DcMotorEx fs,fd,ss,sd;// bs= motor stanga lift si bd= motor dreapta lif
    double viteza_mica=0.4,viteza_mare=1,viteza=1,viteza_medie=0.6,viteza_semi_medie=0.8;

    double fata=0;
    double lateral=0;
    double rotatie=0;
    //double urcare=0;
    boolean D_Up,D_Down,D_Left,D_Right,a,b,y,x,D_Down2,D_Left2,D_Up2,D_Right2,a2,b2,y2,x2,Rb2,Lb2,Rt,Lt;
    double Ltrigger,Rtrigger;

/*
⠀⠀⠀⠀⠀⠀⠀    ⣠⣤⣤⣤⣤⣤⣶⣦⣤⣄⡀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣿⡿⠛⠉⠙⠛⠛⠛⠛⠻⢿⣿⣷⣤⡀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⠋⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⠈⢻⣿⣿⡄⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣸⣿⡏⠀⠀⠀⣠⣶⣾⣿⣿⣿⠿⠿⠿⢿⣿⣿⣿⣄⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠁⠀⠀⢰⣿⣿⣯⠁⠀⠀⠀⠀⠀⠀⠀⠈⠙⢿⣷⡄⠀
⠀⠀⣀⣤⣴⣶⣶⣿⡟⠀⠀⠀⣿⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣷⠀
⠀⢰⣿⡟⠋⠉⣹⣿⡇⠀⠀⠘⣿⣿⣿⣿⣷⣦⣤⣤⣤⣶⣶⣶⣶⣿
⠀⢸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠹⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠀
⠀⣸⣿⡇⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠉⠻⠿⣿⣿⣿⣿⡿⠿⠿⠛⢻⠀⠀
⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣧⠀
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
⠀⣿⣿⠀⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠀⠀
⠀⢿⣿⡆⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡇⠀⠀
⠀⠸⣿⣧⡀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠃⠀⠀
⠀⠀⠛⢿⣿⣿⣿⣿⣇⠀⠀⠀⠀⠀⣰⣿⣿⣷⣶⣶⣶⣶⠶⠀⢠⣿
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⣽⣿⡏⠁⠀⠀⢸⣿⡇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⠀⠀⠀⣿⣿⡇⠀⢹⣿⡆⠀⠀⠀⣸⣿⠇⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢿⣿⣦⣄⣀⣠⣴⣿⣿⠁⠀⠈⠻⣿⣿⣿⣿⡿⠏⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠈⠛⠻⠿⠿⠿⠿⠋⠁⠀⠀
 */

    public void Miscare() {

        if (Rt)
            viteza = viteza_mare;
        else if (Lt)
            viteza = viteza_mica;
        if (y)
            viteza = viteza_medie;
        if (x)
            viteza = viteza_semi_medie;


        fata = Range.clip(-gamepad1.left_stick_y, -viteza, viteza);
        lateral = Range.clip(gamepad1.right_stick_x, -viteza, viteza);
        rotatie = Range.clip(gamepad1.left_stick_x, -viteza, viteza);

        fs.setPower(rotatie + (fata + lateral));//stanga
        fd.setPower(-rotatie + (fata - lateral));
        ss.setPower(rotatie + (fata - lateral));//stanga
        sd.setPower(-rotatie + (fata + lateral));


    }

    @Override
    public void runOpMode()     throws InterruptedException {

        fs = hardwareMap.get(DcMotorEx.class,"leftFront");
        fd = hardwareMap.get(DcMotorEx.class,"rightFront");
        ss = hardwareMap.get(DcMotorEx.class,"leftRear");
        sd = hardwareMap.get(DcMotorEx.class,"rightRear");
//



        //Encoders & Modes


        fs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sd.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        fs.setDirection(DcMotor.Direction.REVERSE);
        ss.setDirection(DcMotor.Direction.REVERSE);
        fd.setDirection(DcMotor.Direction.FORWARD);//il faci sa se invarta invers
        sd.setDirection(DcMotor.Direction.FORWARD);

        sd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()&&!isStopRequested())
        {
            D_Up=gamepad1.dpad_up;
            D_Down=gamepad1.dpad_down;
            D_Left=gamepad1.dpad_left;
            D_Right=gamepad1.dpad_right;
            a=gamepad1.a;
            b= gamepad1.b;
            y= gamepad1.y;
            Rt = gamepad1.right_bumper;
            Lt = gamepad1.left_bumper;

            Miscare();

        }

    }


}


