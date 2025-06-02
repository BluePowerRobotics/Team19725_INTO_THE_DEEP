package org.firstinspires.ftc.teamcode.Controllers;

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
        WAITING, SCANNING, DOWNING, TAKING, UPPING, PUTTING;
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
        WAITING, PREPARING, INSTALLING;
        private static final INSTALL_RUNMODE[] VALUES = values();

        public INSTALL_RUNMODE next() {
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }

    INSTALL_RUNMODE INSTALL_MODE;
}
