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


@TeleOp(name = "MecanumSolo", group = "A")
public class MecanumSolo extends LinearOpMode {
    private ElapsedTime     timer = new ElapsedTime();
    private double lastError=0;

    DcMotorEx fs,fd,ss,sd,scripS;// bs= motor stanga lift si bd= motor dreapta lift
    Servo servoGheara;
    RevTouchSensor hall, limiter;
    double  Kp=7,
            Ki=0,
            Kd=0,
            sum=0;
    double viteza_mica=0.4,viteza_mare=1,viteza=1,bratSpeed=1;
    int bratPosition;
    boolean buttons_mode=false, manual_mode=false;
    double fata=0;
    double lateral=0;
    double rotatie=0;
    //double urcare=0;
    boolean D_Up,D_Down,D_Left,a,b,y,D_Down1,D_Left1,D_Up1,D_Right1,a1,b1,y1,x1,Rb1,Lb1,Rt,Lt;
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

        if(y1)
        {
            scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scripS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            buttons_mode=false;
        }
        if (x1){
            buttons_mode=true;
        }
        if(D_Up1)
            Level3();
        if(D_Left1)
            Level2();
        if(D_Down1)
            Level1();
        if(D_Right1)
            Level0();

        fata=Range.clip(-gamepad1.left_stick_y,-viteza,viteza);
        lateral=Range.clip(gamepad1.right_stick_x,-viteza,viteza);
        rotatie=Range.clip(gamepad1.left_stick_x,-viteza,viteza);

//        fs.setPower(rotatie+(fata+lateral));//stanga
        fd.setPower(-rotatie+(fata-lateral));
        ss.setPower(rotatie+(fata-lateral));//stanga
        sd.setPower(-rotatie+(fata+lateral));


    }



    @Override
    public void runOpMode()     throws InterruptedException {

   //     fs = hardwareMap.get(DcMotorEx.class,"leftFront");
        fd = hardwareMap.get(DcMotorEx.class,"rightFront");
        ss = hardwareMap.get(DcMotorEx.class,"leftRear");
        sd = hardwareMap.get(DcMotorEx.class,"rightRear");
        scripS = hardwareMap.get(DcMotorEx.class, "liftStanga");

        hall =hardwareMap.get(RevTouchSensor.class,"hall");
        limiter = hardwareMap.get(RevTouchSensor.class, "limiter");

        servoGheara = hardwareMap.get(Servo.class, "gheara");



        //Encoders & Modes
        sd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ss.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   //     fs.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   fs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        sd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ss.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //    fs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //brat
        scripS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
            D_Up1=gamepad1.dpad_up;
            D_Down1=gamepad1.dpad_down;
            D_Left1=gamepad1.dpad_left;
            D_Right1=gamepad1.dpad_right;
            a1=gamepad1.a;
            Rb1=gamepad1.right_stick_button;
            Lb1=gamepad1.left_stick_button;
            b1= gamepad1.b;
            x1=gamepad1.x;
            y1=gamepad1.y;
            Ltrigger=gamepad1.left_trigger;
            Rtrigger=gamepad1.right_trigger;


            boolean y=gamepad1.y;
            if(!limiter.isPressed()||!hall.isPressed())
                Miscare();
            Telemntry();
            if(buttons_mode==true){
                setPosition(bratPosition,bratSpeed);
            }
            else {
                Scripete();
            }
            Gheara();
            Limit();


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
        if(a1){
            ReleaseElement();
            sleep(300);

        }
        if(b1){
            CatchElement();
            sleep(50);
        }
        if(isPressedHall()){

            CatchElement();
        }
        if(isPressedHall())
            CatchElement();
    }






    public void Telemntry(){
        telemetry.addData("Pos motor stanga",scripS.getCurrentPosition());
        telemetry.addData("Positie Servo: ", servoGheara.getPosition());
        telemetry.addData("Is Limiter pressed?",isPressedLimiter());
        telemetry.addData("Button Mode:", buttons_mode);
        telemetry.update();
    }

    public void Scripete(){
        if(Rtrigger!=0 && scripS.getCurrentPosition()<6790)
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



    public boolean isPressedHall(){
        return hall.isPressed();
    }
    public boolean isPressedLimiter(){return limiter.isPressed();}
    private void ReleaseElement(){
        servoGheara.setPosition(0.8);
    }
    private void CatchElement(){
        servoGheara.setPosition(0.4);
    }
    private double PidChica(double reference,double state){
        double error =reference-state;
        sum+=error*timer.seconds();
        double derivative = (error-lastError)/timer.seconds();
        lastError=error;

        timer.reset();
        double output =(error*Kp)+(derivative*Kd)+(sum*Ki);
        return output;
    }

    public void  Limit (){
        if (limiter.isPressed()){
            scripS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            scripS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



}


