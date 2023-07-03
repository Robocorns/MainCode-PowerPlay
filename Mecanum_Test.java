package Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Mecanum_Test", group = "A")
public class Mecanum_Test extends LinearOpMode {
    private ElapsedTime     timer = new ElapsedTime();
    DcMotorEx fs,fd,ss,sd,bs,bd;// bs= motor stanga lift si bd= motor dreapta lift
    Servo xd;//xd=servou care inchide/deschide ghiara
    double viteza_mica=0.4,viteza_medie=0.8,viteza_mare=1,viteza=1,bspos,bdpos,x=0.2;
    public static double Kp=0;//trb reglat
    public static double Ki=0;//trb reglat
    public static double Kd=0;//trb reglat
    static final double COUNTS_PER_MOTOR_REV = 1120;    // Rev motor Ticks
    static final double DRIVE_GEAR_REDUCTION = 0.75;     // ???? geared up => >1
    static final double WHEEL_DIAMETER_CM = 10;     // Calculam circumferinta
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * Math.PI);
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
⠀⣿⣿⠁⠀⠀⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣧⠀⠀
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

    public void Miscare(){
        double fata=0;
        double lateral=0;
        double rotatie=0;
        //double urcare=0;
        boolean D_Up,D_Down,D_Left,a,b,y,max,med,min,oc;
        D_Up=gamepad1.dpad_up;
        D_Down=gamepad1.dpad_down;
        D_Left=gamepad1.dpad_left;
        a=gamepad1.a;
        b= gamepad1.b;
        y= gamepad1.y;
        //gamepad2
        max=gamepad2.right_bumper;//max= inaltimea maxima pt a pune conul pe batul mare
        med=gamepad2.left_bumper;//med= inaltimea medie pt a pune conul pe batul mediu
        min=gamepad2.y;//min=inaltimea minima pt a pune conul pe batul mic
        oc=gamepad2.a;//deschidere/inchidere ghiara
        if(D_Up)
            viteza=viteza_mare;
        if(D_Down)
            viteza=viteza_mica;
        if(D_Left)
        {
            viteza=viteza_medie;
        }
        if(max)
        {

        }
        if(med)
        {

        }
        if(min)
        {

        }
        if(oc)
        {

        }

        fata=Range.clip(-gamepad1.left_stick_y,-viteza,viteza);
        lateral=Range.clip(gamepad1.right_stick_x,-viteza,viteza);
        rotatie=Range.clip(gamepad1.left_stick_x,-viteza,viteza);

        //   fata=-gamepad1.left_stick_y;
        //   lateral=gamepad1.left_stick_x;
        //   rotatie=gamepad1.right_stick_x;
        //  formule pt miscare roti
        fs.setPower(rotatie+(fata+lateral));//stanga
//care ti autonomia?

        //acm am incerat TurnTest si FollowerPIDTuner si nu merg?
        //yup


    }



    @Override
    public void runOpMode()     throws InterruptedException {

        fs = hardwareMap.get(DcMotorEx.class,"leftFront"); //facem referinta la motoarele din config
        fd = hardwareMap.get(DcMotorEx.class,"rightFront");
        ss = hardwareMap.get(DcMotorEx.class,"leftRear");
        sd = hardwareMap.get(DcMotorEx.class,"rightRear");
        //brat
        //bs = hardwareMap.get(DcMotorEx.class, "scripS");
        ///   bd = hardwareMap.get(DcMotorEx.class, "scripD");
        //brat


        //Encoders & Modes
        fs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //brat
//        bs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //brat

        //fs.setDirection(DcMotor.Direction.FORWARD);//definim cum se invart motoarele
        fs.setDirection(DcMotor.Direction.REVERSE);
        ss.setDirection(DcMotor.Direction.REVERSE);
        fd.setDirection(DcMotor.Direction.FORWARD);//il faci sa se invarta invers
        sd.setDirection(DcMotor.Direction.FORWARD);

        //brat
//        bs.setDirection(DcMotor.Direction.FORWARD);
//        bd.setDirection(DcMotor.Direction.REVERSE);
        //brat
        sd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//le spunem ce sa faca cand nu primesc putere
        fs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//in cazul  brake se forteaza oprirea


        //brat
//        bs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        //brat

        waitForStart();
        fd.setPower(0);
        fs.setPower(0);
        //brat
//        bs.setPower(0);
//        bd.setPower(0);
//        //brat
        sd.setPower(0);
        ss.setPower(0);

        waitForStart();

        while(opModeIsActive()&&!isStopRequested())
        {
            boolean y=gamepad1.y;
            Miscare();


        }

    }
}


