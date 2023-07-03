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

import org.firstinspires.ftc.teamcode.R;


@TeleOp(name = "MecanumGINV", group = "A")
public class MecanumGINV extends LinearOpMode {
    private ElapsedTime     timer = new ElapsedTime();
    private double lastError=0;

    DcMotorEx fd,ss,sd,scripS;// de pus la loc fs de la fata stanga
    Servo servoGheara;
    RevTouchSensor hall, limiter;

    double viteza_mica=0.4,viteza_mare=1,viteza=1,viteza_medie=0.7,bratSpeed=1;
    int bratPosition;
    boolean buttons_mode=false, manual_mode=false;
    double fata=0;
    double lateral=0;
    double rotatie=0;
    //double urcare=0;
    boolean D_Up,D_Down,D_Left,a,b,y,D_Down2,D_Left2,D_Up2,D_Right2,a2,b2,y2,x2,Rb2,Lb2,Rt,Lt;
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

    public void Miscare(){

        if(Rt)
            viteza=viteza_mare;
        else if (Lt)
            viteza=viteza_mica;
        if(y)
            viteza=viteza_medie;

        if(y2)
        {
            scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scripS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            buttons_mode=false;
        }
        if (x2){
            buttons_mode=true;
        }
        if(D_Up2)
            Level3();
        if(D_Left2)
            Level2();
        if(D_Down2)
            Level1();
        if(D_Right2)
            Level0();

        fata=Range.clip(-gamepad1.left_stick_y,-viteza,viteza);
        lateral=Range.clip(gamepad1.right_stick_x,-viteza,viteza);
        rotatie=Range.clip(gamepad1.left_stick_x,-viteza,viteza);

     //   fs.setPower(rotatie+(fata+lateral));//stanga
        fd.setPower(-rotatie+(fata-lateral));
        ss.setPower(rotatie+(fata-lateral));//stanga
        sd.setPower(-rotatie+(fata+lateral));


    }



    @Override
    public void runOpMode()     throws InterruptedException {

      //  fs = hardwareMap.get(DcMotorEx.class,"leftFront");
        fd = hardwareMap.get(DcMotorEx.class,"rightFront");
        ss = hardwareMap.get(DcMotorEx.class,"leftRear");
        sd = hardwareMap.get(DcMotorEx.class,"rightRear");
        scripS = hardwareMap.get(DcMotorEx.class, "lift");

        hall =hardwareMap.get(RevTouchSensor.class,"hall");
        limiter = hardwareMap.get(RevTouchSensor.class, "limiter");
//
        servoGheara = hardwareMap.get(Servo.class, "gheara");



        //Encoders & Modes
        sd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ss.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //brat
        scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scripS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // fs.setDirection(DcMotor.Direction.REVERSE);
        ss.setDirection(DcMotor.Direction.REVERSE);
        fd.setDirection(DcMotor.Direction.FORWARD);//il faci sa se invarta invers
        sd.setDirection(DcMotor.Direction.FORWARD);

        //brat
          scripS.setDirection(DcMotor.Direction.REVERSE);




        //brat
        scripS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // fs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while(opModeIsActive()&&!isStopRequested())
        {
            D_Up=gamepad1.dpad_up;
            D_Down=gamepad1.dpad_down;
            D_Left=gamepad1.dpad_left;
            a=gamepad1.a;
            b= gamepad1.b;
            y= gamepad1.y;
            Rt = gamepad1.right_bumper;
            Lt = gamepad1.left_bumper;
            //gamepad2
            D_Up2=gamepad2.dpad_up;
            D_Down2=gamepad2.dpad_down;
            D_Left2=gamepad2.dpad_left;
            D_Right2=gamepad2.dpad_right;
            a2=gamepad2.a;
            Rb2=gamepad2.right_stick_button;
            Lb2=gamepad2.left_stick_button;
            b2= gamepad2.b;
            x2=gamepad2.x;
            y2=gamepad2.y;
            Ltrigger=gamepad2.left_trigger;
            Rtrigger=gamepad2.right_trigger;

//
//            boolean y=gamepad1.y;
                Miscare();
                Scripete();
                Limit();
                Gheara();
                if(gamepad1.y)
                    DisableEncoder();
        }

    }


    public void Level3(){
        bratPosition = 3900;
    }

    public void Level2(){
        bratPosition = 2700;
    }

    public void Level1(){
        bratPosition = 1400;
    }
    public void Level0(){
        bratPosition = 0;
    }

    public void setPosition(int pos, double speed){
        scripS.setTargetPosition(pos);
        scripS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        scripS.setPower(speed);
    }

    public void Gheara(){
        if(a2){
            ReleaseElement();
            sleep(200);

        }
        if(b2){
            CatchElement();
            sleep(50);
        }
        if(Hall()){
            CatchElement();
        }

    }






    public void Telemntry(){
        telemetry.addData("Pos motor stanga",scripS.getCurrentPosition());
        telemetry.addData("Positie Servo: ", servoGheara.getPosition());
        telemetry.addData("Is Limiter pressed?",isPressedLimiter());
        telemetry.addData("Button Mode:", buttons_mode);
        telemetry.update();
    }

    public void Scripete()//misca scripetele sus/jos in functie de trigger
    {
        if(Rtrigger!=0 && scripS.getCurrentPosition()<6790)//upper limit
        {
            scripS.setPower(Rtrigger);
        }
        else if (Ltrigger!=0 && !limiter.isPressed()){

            scripS.setPower(-Ltrigger);
        }

        else {
            scripS.setPower(0);
        }
    }



    public boolean Hall(){
        return hall.isPressed();
    }
    public boolean isPressedLimiter(){return limiter.isPressed();}
    private void ReleaseElement(){
        servoGheara.setPosition(0.8);
    }
    private void CatchElement(){
        servoGheara.setPosition(0.4);
    }
    void DisableEncoder(){
        scripS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void  Limit (){
        if (limiter.isPressed()){
            scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scripS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



}


