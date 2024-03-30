package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PrintCommand extends Command {
    private final String text;
    private final double time;
    private boolean isFinished = false;
    private Timer timer;

    public PrintCommand(String text, double time)  {
        this.text = text;
        this.time = time;

    }

    @Override
    public void initialize(){
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.get() < time){
            System.out.println(text);
        }else{
            System.out.println("Done");
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished(){
        return isFinished;
    }
}
