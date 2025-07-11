package org.firstinspires.ftc.teamcode.Controllers;
//控制机器人状态
public class RobotStates {
    // 单例实例
    private static RobotStates instance;

    public static synchronized RobotStates getInstance() {
        if (instance == null) {
            instance = new RobotStates();
        }
        return instance;
    }


    public enum RUNMODE {
        HIGH_CHAMBER, LOW_CHAMBER, HIGH_BASKET, LOW_BASKET;
        // 缓存枚举数组以避免重复调用values()
        private static final RUNMODE[] VALUES = values();

        public RUNMODE next() {
            // 计算下一个位置（循环）
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }

    private RUNMODE MODE = RUNMODE.HIGH_CHAMBER;

    public RUNMODE getMODE() {
        return MODE;
    }

    public void setMODE(RUNMODE MODE) {
        this.MODE = MODE;
    }


    private boolean AUTO = false;

    public boolean isAUTO() {
        return AUTO;
    }

    public void setAUTO(boolean AUTO) {
        this.AUTO = AUTO;
    }


    private boolean CLIMBING = false;

    public boolean isCLIMBING() {
        return CLIMBING;
    }

    public void setCLIMBING(boolean CLIMBING) {
        this.CLIMBING = CLIMBING;
    }


    public enum OUTPUT_RUNMODE {
        WAITING, DOWNING, TAKING, UPPING, PUTTING;
        private static final OUTPUT_RUNMODE[] VALUES = values();

        public OUTPUT_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }

    OUTPUT_RUNMODE OUTPUT_MODE;

    public enum INTAKE_RUNMODE {
        EXTENDING, SHORTENING, TAKING, PUTTING_INSTALL, PUTTING_OUTPUT, WAITING;
        private static final INTAKE_RUNMODE[] VALUES = values();

        public INTAKE_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }

    INTAKE_RUNMODE INTAKE_MODE;

    public enum INSTALL_RUNMODE {
        WAITING, EATING, INSTALLING,BACKING;
        private static final INSTALL_RUNMODE[] VALUES = values();

        public INSTALL_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }

    INSTALL_RUNMODE INSTALL_MODE ;


    public enum HIGH_CHAMBER_RUNMODE {
        INTAKE_ACTION_1, INTAKE_ACTION_2, INTAKE_ACTION_3, INSTALL_ACTION_1, INSTALL_ACTION_2, OUTPUT_ACTION_1, OUTPUT_ACTION_2,OUTPUT_ACTION_3, OUTPUT_ACTION_4;
        private static final HIGH_CHAMBER_RUNMODE[] VALUES = values();

        public HIGH_CHAMBER_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    HIGH_CHAMBER_RUNMODE RobotMode_HIGH_CHAMBER;
    public void setRobotMode(HIGH_CHAMBER_RUNMODE RobotMode_HIGH_CHAMBER) {
        this.RobotMode_HIGH_CHAMBER = RobotMode_HIGH_CHAMBER;
        setRobotMode();
    }
    public void setRobotMode(){
        if(MODE == RUNMODE.HIGH_CHAMBER) {
            switch (RobotMode_HIGH_CHAMBER) {
                case INTAKE_ACTION_1:
                    INTAKE_MODE = INTAKE_RUNMODE.EXTENDING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.UPPING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
                case INTAKE_ACTION_2:
                    INTAKE_MODE = INTAKE_RUNMODE.TAKING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.WAITING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
                case INTAKE_ACTION_3:
                    INTAKE_MODE = INTAKE_RUNMODE.SHORTENING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.WAITING;
                    break;
                case INSTALL_ACTION_1:
                    INTAKE_MODE = INTAKE_RUNMODE.PUTTING_INSTALL;
                    OUTPUT_MODE = OUTPUT_RUNMODE.WAITING;
                    INSTALL_MODE = INSTALL_RUNMODE.EATING;
                    break;
                case INSTALL_ACTION_2:
                    INTAKE_MODE = INTAKE_RUNMODE.WAITING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.DOWNING;
                    INSTALL_MODE = INSTALL_RUNMODE.INSTALLING;
                    break;
                case OUTPUT_ACTION_1:
                    INTAKE_MODE = INTAKE_RUNMODE.PUTTING_OUTPUT;
                    OUTPUT_MODE = OUTPUT_RUNMODE.WAITING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
                case OUTPUT_ACTION_2:
                    INTAKE_MODE = INTAKE_RUNMODE.WAITING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.TAKING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
                case OUTPUT_ACTION_3:
                    INTAKE_MODE = INTAKE_RUNMODE.WAITING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.UPPING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
                case OUTPUT_ACTION_4:
                    INTAKE_MODE = INTAKE_RUNMODE.EXTENDING;
                    OUTPUT_MODE = OUTPUT_RUNMODE.PUTTING;
                    INSTALL_MODE = INSTALL_RUNMODE.WAITING;
                    break;
            }
        }
    }
    public void updateRobotMode() {
        if(true){
            // 这里可以添加条件判断来决定何时更新状态
            // 例如，如果需要在某个特定条件下更新状态，可以在这里实现

            setRobotMode(RobotMode_HIGH_CHAMBER.next());
        }
    }
}
