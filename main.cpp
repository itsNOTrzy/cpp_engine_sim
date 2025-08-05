#define _CRT_SECURE_NO_WARNINGS
#include <graphics.h>
#include <conio.h>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>

// 引擎状态枚举
enum EngineState {
    ENGINE_OFF,
    ENGINE_STARTING,
    ENGINE_RUNNING,
    ENGINE_STOPPING
};

// 异常类型（14种异常描述）
enum FaultType {
    NO_FAULT = 0,

    // 传感器异常类
    N1S1_FAIL,   // 单个转速传感器全故障
    N1S2_FAIL,   // 单发转速传感器全故障
    EGTS1_FAIL,  // 单个EGT传感器全故障
    EGTS2_FAIL,  // 单发EGT传感器全故障
    N1S_FAIL,    // 双发转速传感器全故障
    EGTS_FAIL,   // 双发EGT传感器全故障

    // 燃油类异常
    LOW_FUEL,
    FUELS_FAIL,
    OVER_FF,   // 燃油流速超过 50
    // 转速异常
    OVER_SPD1, // N1>105
    OVER_SPD2, // N1>120
    // 温度异常
    OVER_TEMP1, // 启动中T>850
    OVER_TEMP2, // 启动中T>1000
    OVER_TEMP3, // 稳态中T>950
    OVER_TEMP4  // 稳态中T>1100
};

// 告警信息
struct AlertInfo {
    FaultType type;
    std::string text;
    COLORREF color;
    DWORD last_trigger_time;
};

// 告警框（按钮）
struct FaultDisplay {
    std::string text;
    int x, y, w, h; // 位置
    FaultType ft;
    bool isActive = false; // 按钮激活状态，默认为未激活
};

// 全局运行标志
static bool g_quit = false;

// 左、右引擎数据
struct EngineData {
    double N1;    // 转速百分比, 0~125
    double T;     // 温度 ℃, -5~1200
    double FF;    // Fuel Flow 燃油流速 0~50
    bool n1Sensor1Fail;
    bool n1Sensor2Fail;
    bool egtSensor1Fail;
    bool egtSensor2Fail;
    double targetN1;
    double targetT;
    double targetFF;
};

// 燃油数据
struct FuelData {
    double C; // 燃油余量 0~20000
    bool fuelSensorFail;
};

// 程序状态
struct ProgramState {
    EngineState state;
    bool started;
    bool stabilized;
    bool stopped;
    bool run_light_on;   // run灯状态
    bool start_light_on; // start灯状态
    bool state_changed; // 状态是否刚刚改变
    // 推力调整按钮计数
    double thrustAdjust;
};

// 全局变量
static ProgramState g_state;
static EngineData g_leftEngine, g_rightEngine;
static FuelData g_fuel;
static DWORD g_startTime;
static DWORD g_stopTime;
static std::ofstream g_dataFile;
static std::ofstream g_logFile;

static int g_width = 600;
static int g_height = 650;

// 按钮的坐标和尺寸
struct Button {
    int x, y, w, h;
    std::string text;
};
static Button btnStart = { 250, 300, 80, 40, "START" };
static Button btnStop = { 350, 300, 80, 40, "STOP" };
static Button btnUp = { 450, 200, 30, 30, "▲" };
static Button btnDown = { 450, 240, 30, 30, "▼" };

// 状态告警框的布局
const int g_faultX = 50;
const int g_faultY = 350;
static std::vector<FaultDisplay> g_faultDisplays = {
    {"N1S1Fail",   g_faultX,g_faultY,80,25,N1S1_FAIL},
    {"N1S2Fail",   g_faultX + 100,g_faultY,80,25,N1S2_FAIL},
    {"EGTS1Fail",  g_faultX + 200,g_faultY,80,25,EGTS1_FAIL},
    {"EGTS2Fail",  g_faultX + 300,g_faultY,80,25,EGTS2_FAIL},
    {"N1SFail",    g_faultX,g_faultY + 30,80,25,N1S_FAIL},
    {"EGTSFail",   g_faultX + 100,g_faultY + 30,80,25,EGTS_FAIL},
    {"LowFuel",    g_faultX + 200,g_faultY + 30,80,25,LOW_FUEL},
    {"FuelSFail",  g_faultX + 300,g_faultY + 30,80,25,FUELS_FAIL},
    {"OverSpd1",   g_faultX,g_faultY + 60,80,25,OVER_SPD1},
    {"OverSpd2",   g_faultX + 100,g_faultY + 60,80,25,OVER_SPD2},
    {"OverFF",     g_faultX + 200,g_faultY + 60,80,25,OVER_FF},
    {"OverTemp1",  g_faultX,g_faultY + 90,80,25,OVER_TEMP1},
    {"OverTemp2",  g_faultX + 100,g_faultY + 90,80,25,OVER_TEMP2},
    {"OverTemp3",  g_faultX + 200,g_faultY + 90,80,25,OVER_TEMP3},
    {"OverTemp4",  g_faultX + 300,g_faultY + 90,80,25,OVER_TEMP4}
};

// 告警记录(5秒内同一种不重复记录)
static std::vector<AlertInfo> g_alerts;

// 角度转换：N1与T的指示范围为0°~210°
// 假定0%对应0°，满量程(如N1=125)对应210°。
inline double valueToAngle(double val, double maxVal = 125.0) {
    if (val < 0) val = 0;
    if (val > maxVal) val = maxVal;
    double ratio = val / maxVal;
    return 210.0 * ratio;
}

inline double valueToAngleT(double val, double maxVal = 1200.0) {
    // EGT同理0~1200°C -> 0~210°
    if (val < -5) val = -5;
    if (val > 1200) val = 1200;
    double ratio = (val + 5) / (1200 + 5); // shift to start from 0
    return 210.0 * ratio;
}

// 根据异常类型返回颜色
COLORREF getColorForFault(FaultType ft, bool sensorValueInvalid = false) {
    // 根据异常严重程度返回颜色
    // 无效值用灰色表示
    // 警告值红色，警戒值琥珀色，正常白色
    switch (ft) {
    case N1S1_FAIL:
    case EGTS1_FAIL:
        return RGB(255, 255, 255); //白色警告（传感器单个故障）
    case EGTS2_FAIL:
    case N1S2_FAIL: // 单发传感器全废，算警戒
    case LOW_FUEL:
    case OVER_FF:
    case OVER_SPD1:
    case OVER_TEMP1:
    case OVER_TEMP3:
        return RGB(255, 140, 0); // 琥珀色
    case N1S_FAIL:
    case EGTS_FAIL:
    case FUELS_FAIL:
    case OVER_SPD2:
    case OVER_TEMP2:
    case OVER_TEMP4:
        return RGB(255, 0, 0); // 红色
    default:
        if (sensorValueInvalid)
            return RGB(128, 128, 128); //灰色
        return RGB(255, 255, 255); //正常白
    }
}

// 根据故障类型获取文本
std::string faultTypeToString(FaultType ft) {
    switch (ft) {
    case N1S1_FAIL: return "One N1 Sensor Fail";
    case N1S2_FAIL: return "One Engine N1 Sensor Fail";
    case EGTS1_FAIL: return "One EGT Sensor Fail";
    case EGTS2_FAIL: return "One Engine EGT Sensor Fail";
    case N1S_FAIL: return "All N1 Sensor Fail";
    case EGTS_FAIL: return "All EGT Sensor Fail";
    case LOW_FUEL: return "Low Fuel 1000";
    case FUELS_FAIL: return "Fuel Sensor Fail";
    case OVER_FF: return "Over Fuel Flow 50";
    case OVER_SPD1: return "Over Speed 105";
    case OVER_SPD2: return "Over Speed 120";
    case OVER_TEMP1: return "Over Temperature 850 when STARTING";
    case OVER_TEMP2: return "Over Temperature 1000 when STARTING";
    case OVER_TEMP3: return "Over Temperature 950 when RUNNING";
    case OVER_TEMP4: return "Over Temperature 1100 when RUNNING";
    default: return "";
    }
}

// 将当前故障记录到 log 文件
void logFault(FaultType ft) {
    if (ft == NO_FAULT) return;
    DWORD now = GetTickCount();
    for (auto& a : g_alerts) {
        if (a.type == ft) {
            // 若5s内已记录过，则不重复记录
            if (now - a.last_trigger_time < 5001) return;
            a.last_trigger_time = now;
            break;
        }
    }

    // 找不到则新加入
    AlertInfo ai;
    ai.type = ft;
    ai.text = faultTypeToString(ft);
    ai.color = getColorForFault(ft);
    ai.last_trigger_time = now;
    g_alerts.push_back(ai);

    // 写入log
    DWORD runningTime = now - g_startTime;
    g_logFile << runningTime << "ms: " << ai.text << std::endl;
    g_logFile.flush();
}

// 绘制表盘背景和指针
void drawGauge(int centerX, int centerY, double angle, double value, bool isEGT, FaultType ft = NO_FAULT) {
    int radius = 60;
    double sweepRad = angle * 3.1415926 / 180.0;
    // 根据故障类型或异常判断颜色
    COLORREF col = RGB(255, 255, 255);
    // 若有故障则颜色变化
    if (ft != NO_FAULT) {
        col = getColorForFault(ft);
    }
    setlinecolor(WHITE);
    line(centerX, centerY, centerX + radius, centerY);
    arc(centerX - radius, centerY - radius, centerX + radius, centerY + radius, -3.66519137, 0);
    setbkmode(TRANSPARENT);
    settextstyle(20, 0, "Consolas");
    settextcolor(col);
    // 绘制数值文本
    char buf[32];
    if (ft != NO_FAULT) {
        // 如果该故障导致无效值，用"--"
        if (ft == N1S2_FAIL || ft == N1S_FAIL || ft == EGTS2_FAIL || ft == EGTS_FAIL) {
            sprintf(buf, "--");
            outtextxy(centerX + 10, centerY - 20, buf);
            return;
        }
        else {
            if (isEGT) sprintf(buf, "%d", (int)value);
            else sprintf(buf, "%.1f", value);
        }
    }
    else {
        if (isEGT) sprintf(buf, "%d", (int)value);
        else sprintf(buf, "%.1f", value);
    }
    outtextxy(centerX + 10, centerY - 20, buf);
    //扇形
    if ((int)angle == 0) return;
    setlinecolor(col);
    setfillcolor(col);
    fillpie(centerX - radius, centerY - radius, centerX + radius, centerY + radius, -sweepRad, 0);
}
// 绘制按钮
void drawButton(const Button& btn, bool pressed = false) {
    setlinecolor(WHITE);
    setfillcolor(pressed ? RGB(200, 200, 200) : RGB(100, 100, 255));
    solidrectangle(btn.x, btn.y, btn.x + btn.w, btn.y + btn.h);
    settextstyle(20, 0, "Consolas");
    settextcolor(BLACK);
    setbkmode(TRANSPARENT);
    outtextxy(btn.x + 5, btn.y + 5, btn.text.c_str());
}
// 绘制状态
void drawStatusBoxes() {
    // 若start灯亮，画蓝色背景
    // run灯亮，画绿色背景
    // 若不亮，画深色背景
    int xstart = 50, ystart = 300;
    settextstyle(20, 0, "Consolas");
    setbkmode(OPAQUE);

    setfillcolor(g_state.start_light_on ? RGB(0, 0, 255) : RGB(50, 50, 50));
    solidrectangle(xstart, ystart, xstart + 60, ystart + 25);
    settextcolor(BLACK);
    setbkmode(TRANSPARENT);
    outtextxy(xstart + 5, ystart + 2, "START");

    setfillcolor(g_state.run_light_on ? RGB(0, 255, 0) : RGB(50, 50, 50));
    solidrectangle(xstart + 70, ystart, xstart + 70 + 60, ystart + 25);
    settextcolor(BLACK);
    setbkmode(TRANSPARENT);
    outtextxy(xstart + 75, ystart + 2, "RUN");
}
// 绘制燃油流速
void drawFFInfo(double ff, FaultType ft) {
    int x = 500, y = 100;
    settextstyle(20, 0, "Consolas");
    setbkmode(TRANSPARENT);

    COLORREF col = WHITE;
    settextcolor(col);
    outtextxy(x - 50, y - 20, "Fuel Flow:");
    char buf[32];
    if (ft == OVER_FF) {
        col = getColorForFault(ft);
        settextcolor(col);
        sprintf(buf, "%.0f", ff);
    }
    else {
        sprintf(buf, "%.0f", ff);
    }
    outtextxy(x + 50, y - 20, buf);
}
// 绘制燃油余量
void drawFuelInfo(double c, FaultType ft) {
    int x = 500, y = 100;
    settextstyle(20, 0, "Consolas");
    setbkmode(TRANSPARENT);
    char buf[32];
    COLORREF col = WHITE;
    settextcolor(col);
    outtextxy(x - 50, y + 10, "Fuel:");
    if (ft == FUELS_FAIL) {
        col = getColorForFault(ft);
        settextcolor(col);
        sprintf(buf, "--");
    }
    else if (ft == LOW_FUEL) {
        col = getColorForFault(ft);
        settextcolor(col);
        sprintf(buf, "%.0f", c);
    }
    else {
        sprintf(buf, "%.0f", c);
    }
    outtextxy(x, y + 10, buf);
}

// 绘制警告框
void drawFaultDisplays() {
    for (auto& fd : g_faultDisplays) {
        COLORREF col = fd.isActive ? WHITE : RGB(128, 128, 128); // 激活亮黄，未激活灰色
        setfillcolor(RGB(50, 50, 50));
        solidrectangle(fd.x, fd.y, fd.x + fd.w, fd.y + fd.h);
        settextstyle(15, 0, "Consolas");
        setbkmode(TRANSPARENT);
        settextcolor(col);
        outtextxy(fd.x + 5, fd.y + 5, fd.text.c_str());
    }
}

// 绘制警示文本框
void drawTextArea() {
    // 绘制背景框
    setfillcolor(RGB(50, 50, 50));  // 设置背景色
    solidroundrect(50, 500, 500, 620, 10, 10);  // 绘制背景框

    // 设置文字样式和颜色
    settextstyle(20, 0, "Consolas");
    setbkmode(TRANSPARENT);

    DWORD now = GetTickCount();
    int yOffset = 510; // 初始文字显示的 Y 坐标

    // 遍历当前的告警信息
    std::vector<AlertInfo> activeAlerts;

    // 遍历并过滤超时的告警
    for (auto& alert : g_alerts) {
        if (now - alert.last_trigger_time < 5000) {
            activeAlerts.push_back(alert);
        }
    }

    // 清除所有已经过期的告警
    g_alerts = activeAlerts;

    // 绘制剩余的告警
    for (const auto& alert : g_alerts) {
        // 设置文字颜色
        settextcolor(alert.color);

        // 显示告警文字
        outtextxy(60, yOffset, alert.text.c_str());

        // 调整下一个告警的 Y 坐标，避免覆盖
        yOffset += 25;

        // 如果超出背景框范围，则停止绘制
        if (yOffset > 590) break;
    }
}


// 检查故障并绘制数据
FaultType checkFuelFault() {
    if (g_fuel.fuelSensorFail) {
        return FUELS_FAIL;
    }
    else if (g_fuel.C < 1000 && (g_state.state == ENGINE_RUNNING || g_state.state == ENGINE_STARTING)) {
        if (g_fuel.C <= 0) {
            g_state.state = ENGINE_STOPPING;
            g_stopTime = GetTickCount();
            g_state.thrustAdjust = 0;
        }
        return LOW_FUEL;
    }
    return NO_FAULT;
}
FaultType checkN1Fault(const EngineData& engine) {
    if (engine.n1Sensor1Fail && engine.n1Sensor2Fail) {
        return N1S2_FAIL;
    }
    else if (engine.n1Sensor1Fail || engine.n1Sensor2Fail) {
        return N1S1_FAIL;
    }

    if (engine.N1 > 120) {
        g_state.state = ENGINE_STOPPING;
        g_stopTime = GetTickCount();
        g_state.thrustAdjust = 0;
        return OVER_SPD2;
    }
    else if (engine.N1 > 105) {
        return OVER_SPD1;
    }
    return NO_FAULT;
}
FaultType checkTemperatureFault(const EngineData& engine) {
    if (engine.egtSensor1Fail && engine.egtSensor2Fail) {
        return EGTS2_FAIL; // 单发EGT传感器全部故障
    }
    else if (engine.egtSensor1Fail || engine.egtSensor2Fail) {
        return EGTS1_FAIL; // 单个EGT传感器故障
    }

    if (g_state.state == ENGINE_STARTING) {
        if (engine.T > 1000) {
            g_state.state = ENGINE_STOPPING;
            g_stopTime = GetTickCount();
            g_state.thrustAdjust = 0;
            return OVER_TEMP2;
        }
        else if (engine.T > 850) {
            return OVER_TEMP1;
        }
    }
    else if (g_state.state == ENGINE_RUNNING) {
        if (engine.T > 1100) {
            g_state.state = ENGINE_STOPPING;
            g_stopTime = GetTickCount();
            g_state.thrustAdjust = 0;
            return OVER_TEMP4;
        }
        else if (engine.T > 950) {
            return OVER_TEMP3;
        }
    }
    return NO_FAULT;
}
void checkFault() {
    // 燃油故障
    FaultType fuelFault = checkFuelFault();
    drawFuelInfo(g_fuel.C, fuelFault);
    if (fuelFault != NO_FAULT) {
        logFault(fuelFault);
    }

    if (g_leftEngine.FF > 50 || g_rightEngine.FF > 50) {
        drawFFInfo((g_leftEngine.FF + g_rightEngine.FF) * 0.5, OVER_FF);
        logFault(OVER_FF);
    }
    else drawFFInfo((g_leftEngine.FF + g_rightEngine.FF) * 0.5, NO_FAULT);

    // 左N1
    FaultType leftN1Fault = checkN1Fault(g_leftEngine);
    drawGauge(100, 100, valueToAngle(g_leftEngine.N1), g_leftEngine.N1, false, leftN1Fault);
    if (leftN1Fault != NO_FAULT) {
        logFault(leftN1Fault);
    }
    // 右N1
    FaultType rightN1Fault = checkN1Fault(g_rightEngine);
    drawGauge(300, 100, valueToAngle(g_rightEngine.N1), g_rightEngine.N1, false, rightN1Fault);
    if (rightN1Fault != NO_FAULT) {
        logFault(rightN1Fault);
    }
    // 左EGT
    FaultType leftTempFault = checkTemperatureFault(g_leftEngine);
    drawGauge(100, 200, valueToAngleT(g_leftEngine.T), g_leftEngine.T, true, leftTempFault);
    if (leftTempFault != NO_FAULT) {
        logFault(leftTempFault);
    }
    // 右EGT
    FaultType rightTempFault = checkTemperatureFault(g_rightEngine);
    drawGauge(300, 200, valueToAngleT(g_rightEngine.T), g_rightEngine.T, true, rightTempFault);
    if (rightTempFault != NO_FAULT) {
        logFault(rightTempFault);
    }

    // 两发转速传感器全失败
    if (g_leftEngine.n1Sensor1Fail && g_leftEngine.n1Sensor2Fail &&
        g_rightEngine.n1Sensor1Fail && g_rightEngine.n1Sensor2Fail) {
        drawGauge(100, 100, valueToAngle(g_leftEngine.N1), g_leftEngine.N1, false, N1S_FAIL);
        drawGauge(300, 100, valueToAngle(g_rightEngine.N1), g_rightEngine.N1, false, N1S_FAIL);
        logFault(N1S_FAIL);
        g_state.state = ENGINE_STOPPING;
        g_state.thrustAdjust = 0;
        if (g_leftEngine.N1 <= 1) {
            g_state.state = ENGINE_OFF; // 切换到关闭状态
            g_state.run_light_on = false; // 关闭运行灯
            g_state.state_changed = true;
        }
    }
    // 两发EGT都失败
    if (g_leftEngine.egtSensor1Fail && g_leftEngine.egtSensor2Fail &&
        g_rightEngine.egtSensor1Fail && g_rightEngine.egtSensor2Fail) {
        drawGauge(100, 200, valueToAngleT(g_leftEngine.T), g_leftEngine.T, true, EGTS_FAIL);
        drawGauge(300, 200, valueToAngleT(g_rightEngine.T), g_rightEngine.T, true, EGTS_FAIL);
        logFault(EGTS_FAIL);
        g_state.state = ENGINE_STOPPING;
        g_state.thrustAdjust = 0;
        if (g_leftEngine.N1 <= 1) {
            g_state.state = ENGINE_OFF; // 切换到关闭状态
            g_state.run_light_on = false; // 关闭运行灯
            g_state.state_changed = true;
        }
    }
}

// 初始化
void initData() {
    srand((unsigned)time(NULL));
    g_state.state = ENGINE_OFF;
    g_state.started = false;
    g_state.stabilized = false;
    g_state.stopped = true;
    g_state.run_light_on = false;
    g_state.start_light_on = false;
    g_state.thrustAdjust = 0;

    g_leftEngine = { 0,20,0,false,false,false,false };
    g_rightEngine = { 0,20,0,false,false,false,false };
    g_fuel = { 3000,false };

    g_startTime = GetTickCount();

    g_dataFile.open("data.csv", std::ios::out);
    g_dataFile << "Time(ms),N1_left,N1_right,T_left,T_right,FF_left,FF_right,Fuel\n";
    g_logFile.open("log.txt", std::ios::out);
}

// 数据更新逻辑
void updateData() {
    DWORD now = GetTickCount(); // 5ms一次
    double t = (now - g_startTime) / 1000.0; // seconds
    static double lastC = g_fuel.C;
    static DWORD lastUpdate = now;
    double dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    static double value = 20.0; // 表盘基准值
    static double delta = 0.0;  // 当前变化速率
    static double damping = 0.9; // 阻尼系数（越接近 1 越平滑）
    delta += ((rand() % 201 - 100) / 100.0) * 0.005; // 随机速率调整
    value += delta;
    delta *= damping; // 阻尼减速
    value += (20.0 - value) * 0.05; // 回归基准值
    if (value > 21.5) value = 21.5;
    if (value < 18.5) value = 18.5;

    // 根据状态决定N,T,V变化
    if (g_state.state == ENGINE_OFF) {
        g_state.thrustAdjust == 0;
        g_state.start_light_on = false;
        g_state.run_light_on = false;
        g_leftEngine.N1 = 0;g_rightEngine.N1 = 0;
        g_leftEngine.FF = 0;g_rightEngine.FF = 0;
        g_leftEngine.T = value;
        g_rightEngine.T = g_leftEngine.T + ((rand() % 201 - 100) / 100.0) * 0.005;
    }
    else if (g_state.state == ENGINE_STARTING) {
        // 线性增加阶段
        if (g_leftEngine.N1 < 50 || g_rightEngine.N1 < 50) {
            g_leftEngine.N1 += 10000.0 / 40000 * dt * 100 + (rand() % 3 - 1) * 0.3;
            g_rightEngine.N1 = g_leftEngine.N1;
            g_leftEngine.FF += 5.0 * dt;
            g_rightEngine.FF = g_leftEngine.FF;
            g_leftEngine.T = value;
            g_rightEngine.T = g_leftEngine.T + ((rand() % 201 - 100) / 100.0) * 0.005;
        }
        else {
            // 对数上升阶段
            double x = t - 1;
            double V = 42 * log10(x > 1 ? x : 1) + 10;
            if (V < 0) V = 0;if (V > 50)V = 50;
            double N = 23000 * log10(x > 1 ? x : 1) + 20000; //实际转速N，这里N1 = N/40000*100
            double n1 = N / 400.0;
            if (n1 > 95) {
                g_state.state = ENGINE_RUNNING;
                g_state.stabilized = true;
                g_state.start_light_on = false;
                g_state.run_light_on = true;
            }
            double v = (g_faultDisplays[12].isActive == true)? 1500 : ((g_faultDisplays[11].isActive == true)? 1170 : 900);
            double T = v * log10(x > 1 ? x : 1) + 20;
            g_leftEngine.N1 = n1 + (rand() % 3 - 1) * 0.3; g_rightEngine.N1 = n1 + (rand() % 3 - 1) * 0.3;
            g_leftEngine.FF = V + (rand() % 3 - 1) * 0.03; g_rightEngine.FF = V + (rand() % 3 - 1) * 0.03;
            g_leftEngine.T = T + (rand() % 3 - 1) * 0.3; g_rightEngine.T = T + (rand() % 3 - 1) * 0.3;
        }
    }
    else if (g_state.state == ENGINE_RUNNING) {
        // 稳态阶段
        if (g_state.thrustAdjust == 0) { // 增加推力
            g_leftEngine.N1 += (rand() % 3 - 1) * 0.05;
            g_rightEngine.N1 = g_leftEngine.N1 + (rand() % 3 - 1) * 0.03;
            g_leftEngine.FF += (rand() % 3 - 1) * 0.05;
            g_rightEngine.FF = g_leftEngine.FF + (rand() % 3 - 1) * 0.03;
            g_leftEngine.T += (rand() % 3 - 1) * 0.5;
            g_rightEngine.T = g_leftEngine.T + (rand() % 3 - 1) * 0.03;
        }
        else {
            // 平滑过渡到目标值
            double smoothingFactor = 0.1; // 平滑变化的因子，越小越慢

            // 转速逐步逼近目标值
            g_leftEngine.N1 += (g_leftEngine.targetN1 - g_leftEngine.N1) * smoothingFactor;
            g_rightEngine.N1 = g_leftEngine.N1 + ((rand() % 3 - 1) * 0.05); // 小幅随机波动

            // 燃油流速逐步逼近目标值
            g_leftEngine.FF += (g_leftEngine.targetFF - g_leftEngine.FF) * smoothingFactor;
            g_rightEngine.FF = g_leftEngine.FF + ((rand() % 3 - 1) * 0.03);

            // 温度逐步逼近目标值
            g_leftEngine.T += (g_leftEngine.targetT - g_leftEngine.T) * smoothingFactor;
            g_rightEngine.T = g_leftEngine.T + ((rand() % 3 - 1) * 0.5);

            if (abs(g_leftEngine.targetFF - g_leftEngine.FF) < 0.01
                && abs(g_leftEngine.targetN1 - g_leftEngine.N1) < 0.01
                && abs(g_leftEngine.targetT - g_leftEngine.T) < 0.01) g_state.thrustAdjust = 0;
        }
    }
    else if (g_state.state == ENGINE_STOPPING) {
        g_state.thrustAdjust = 0;
        // 燃油流速直接归零
        g_leftEngine.FF = 0;
        g_rightEngine.FF = 0;

        // 初始化停止状态时的初始值
        static double initialN1 = 0.0; // 停止时的转速起点
        static double initialT = 0.0;  // 停止时的温度起点

        if (g_state.state_changed) {
            g_state.state_changed = false; // 防止多次初始化
            initialN1 = g_leftEngine.N1;   // 记录当前转速
            initialT = g_leftEngine.T;     // 记录当前温度
            g_stopTime = now;              // 记录停止开始时间
        }

        double elapsedTime = (now - g_stopTime) / 1000.0;
        double logFactor = log10(elapsedTime + 1) / log10(4 + 1); // 对数归一化

        // 转速下降
        g_leftEngine.N1 = initialN1 * (1 - logFactor); // 从初始值下降
        g_rightEngine.N1 = g_leftEngine.N1;

        // 温度下降
        g_leftEngine.T = 20 + (initialT - 20) * (1 - logFactor); // 从初始值下降到 20
        g_rightEngine.T = g_leftEngine.T;

        // 停止条件
        if (g_leftEngine.N1 <= 1.0) {
            g_state.state = ENGINE_OFF; // 切换到关闭状态
            g_state.run_light_on = false; // 关闭运行灯
            g_state.state_changed = true;
        }
    }

    // 燃油余量C = 上一时刻C - FF*dt, FF为总流量(简单处理)
    double avgFF = (g_leftEngine.FF + g_rightEngine.FF) * 0.5;
    if (!g_fuel.fuelSensorFail && g_state.state != ENGINE_OFF) {
        g_fuel.C -= avgFF * dt;
        if (g_fuel.C < 0) g_fuel.C = 0;
    }

    // 若N1下降至95以下，run灯熄灭，回升后再次亮起
    if (g_state.stabilized && (g_leftEngine.N1 < 95 && g_rightEngine.N1 < 95)) {
        g_state.run_light_on = false;
    }
    else if (g_state.stabilized && (g_leftEngine.N1 >= 95 || g_rightEngine.N1 >= 95)) {
        g_state.run_light_on = true;
    }

    // 写入数据文件
    g_dataFile << (now - g_startTime) << ","
        << g_leftEngine.N1 << "," << g_rightEngine.N1 << ","
        << g_leftEngine.T << "," << g_rightEngine.T << ","
        << g_leftEngine.FF << "," << g_rightEngine.FF << ","
        << g_fuel.C << "\n";
    g_dataFile.flush();
}

// 检查鼠标点击按钮
void checkMouse() {
    ExMessage msg;
    while (peekmessage(&msg)) {
        if (msg.message == WM_LBUTTONDOWN) {
            int mx = msg.x, my = msg.y;
            // 检测stop按钮
            if (g_state.state == ENGINE_STARTING || ENGINE_RUNNING) {
                if (mx > btnStop.x && mx<btnStop.x + btnStop.w && my>btnStop.y && my < btnStop.y + btnStop.h) {
                    g_state.state = ENGINE_STOPPING;
                    g_stopTime = GetTickCount();
                    g_state.state_changed = true;
                    // Stop优先级最高
                }
            }
            // 检测start按钮
            if (g_state.state == ENGINE_OFF) {
                if (mx > btnStart.x && mx<btnStart.x + btnStart.w && my>btnStart.y && my < btnStart.y + btnStart.h) {
                    g_state.state = ENGINE_STARTING;
                    g_state.started = true;
                    g_state.start_light_on = true;
                    g_state.run_light_on = false;
                    g_startTime = GetTickCount(); //重新计时
                    g_state.state_changed = true;
                }
            }
            if (g_state.state == ENGINE_RUNNING) {
                // 增加推力
                if (mx > btnUp.x && mx<btnUp.x + btnUp.w && my>btnUp.y && my < btnUp.y + btnUp.h) {
                    g_state.thrustAdjust = 1;
                    g_leftEngine.targetN1 = g_leftEngine.N1 * (1.03 + ((rand() % 3) * 0.01)); // 目标转速增加3%-5%
                    g_rightEngine.targetN1 = g_leftEngine.targetN1; // 同步右引擎
                    g_leftEngine.targetFF = g_leftEngine.FF + 1; // 燃油流速目标值增加1
                    g_rightEngine.targetFF = g_leftEngine.targetFF;
                    g_leftEngine.targetT = g_leftEngine.T * (1.03 + ((rand() % 3) * 0.01)); // 温度目标值增加3%-5%
                    g_rightEngine.targetT = g_leftEngine.targetT;
                }
                // 减小推力
                if (mx > btnDown.x && mx<btnDown.x + btnDown.w && my>btnDown.y && my < btnDown.y + btnDown.h) {
                    g_state.thrustAdjust = -1;
                    g_leftEngine.targetN1 = g_leftEngine.N1 * (0.97 - ((rand() % 3) * 0.01)); // 目标转速减少3%-5%
                    g_rightEngine.targetN1 = g_leftEngine.targetN1; // 同步右引擎
                    g_leftEngine.targetFF = g_leftEngine.FF - 1; // 燃油流速目标值减少1
                    g_rightEngine.targetFF = g_leftEngine.targetFF;
                    g_leftEngine.targetT = g_leftEngine.T * (0.97 - ((rand() % 3) * 0.01)); // 温度目标值减少3%-5%
                    g_rightEngine.targetT = g_leftEngine.targetT;
                }
            }
            for (auto& fd : g_faultDisplays) {
                if (mx > fd.x && mx < fd.x + fd.w && my > fd.y && my < fd.y + fd.h) {
                    // 切换按钮状态
                    fd.isActive = !fd.isActive;

                    // 根据激活状态设置故障效果
                    if (fd.isActive) {
                        if (fd.ft == N1S1_FAIL) g_leftEngine.n1Sensor1Fail = true;
                        if (fd.ft == N1S2_FAIL) g_leftEngine.n1Sensor1Fail = g_leftEngine.n1Sensor2Fail = true;
                        if (fd.ft == EGTS1_FAIL) g_leftEngine.egtSensor1Fail = true;
                        if (fd.ft == EGTS2_FAIL) g_leftEngine.egtSensor1Fail = g_leftEngine.egtSensor2Fail = true;
                        if (fd.ft == N1S_FAIL) {
                            g_leftEngine.n1Sensor1Fail = true;
                            g_leftEngine.n1Sensor2Fail = true;
                            g_rightEngine.n1Sensor1Fail = true;
                            g_rightEngine.n1Sensor2Fail = true;
                            g_stopTime = GetTickCount();
                        }
                        if (fd.ft == EGTS_FAIL) {
                            g_leftEngine.egtSensor1Fail = true;
                            g_leftEngine.egtSensor2Fail = true;
                            g_rightEngine.egtSensor1Fail = true;
                            g_rightEngine.egtSensor2Fail = true;
                            g_stopTime = GetTickCount();
                        }
                        if (fd.ft == FUELS_FAIL) g_fuel.fuelSensorFail = true;

                        if (fd.ft == LOW_FUEL) g_fuel.C = 998;
                        if (fd.ft == OVER_FF) g_leftEngine.FF = g_rightEngine.FF = 52;
                        if (fd.ft == OVER_SPD1) g_leftEngine.N1 = g_rightEngine.N1 = 107;
                        if (fd.ft == OVER_SPD2) g_leftEngine.N1 = g_rightEngine.N1 = 122;
                        if (fd.ft == OVER_TEMP3) g_leftEngine.T = g_rightEngine.T = 952;
                        if (fd.ft == OVER_TEMP4) g_leftEngine.T = g_rightEngine.T = 1102;
                    }
                    else {
                        // 取消故障效果（假设重置为正常状态）
                        if (fd.ft == N1S1_FAIL) g_leftEngine.n1Sensor1Fail = false;
                        if (fd.ft == N1S2_FAIL) g_leftEngine.n1Sensor1Fail = g_leftEngine.n1Sensor2Fail = false;
                        if (fd.ft == EGTS1_FAIL) g_leftEngine.egtSensor1Fail = false;
                        if (fd.ft == EGTS2_FAIL) g_leftEngine.egtSensor1Fail = g_leftEngine.egtSensor2Fail = false;
                        if (fd.ft == N1S_FAIL) {
                            g_leftEngine.n1Sensor1Fail = false;
                            g_leftEngine.n1Sensor2Fail = false;
                            g_rightEngine.n1Sensor1Fail = false;
                            g_rightEngine.n1Sensor2Fail = false;
                        }
                        if (fd.ft == EGTS_FAIL) {
                            g_leftEngine.egtSensor1Fail = false;
                            g_leftEngine.egtSensor2Fail = false;
                            g_rightEngine.egtSensor1Fail = false;
                            g_rightEngine.egtSensor2Fail = false;
                        }
                        if (fd.ft == FUELS_FAIL) g_fuel.fuelSensorFail = false;

                        if (fd.ft == LOW_FUEL) g_fuel.C = 3000; // 重置为正常油量
                        if (fd.ft == OVER_FF) g_leftEngine.FF = g_rightEngine.FF = 40;
                        if (fd.ft == OVER_SPD1 || fd.ft == OVER_SPD2) g_leftEngine.N1 = g_rightEngine.N1 = 95; // 正常转速
                        if (fd.ft == OVER_TEMP1 || fd.ft == OVER_TEMP2 || fd.ft == OVER_TEMP3 || fd.ft == OVER_TEMP4)
                            g_leftEngine.T = g_rightEngine.T = 730; // 正常温度
                    }
                }
            }
        }
        else if (msg.message == WM_CLOSE) {
            g_quit = true;
        }
    }
}

// 绘制主界面
void drawUI() {
    setfillcolor(BLACK);
    solidrectangle(0, 0, g_width, g_height);

    // 当前故障检查并绘制数据
    checkFault();
    // 绘制告警框
    drawFaultDisplays();
    // 绘制四个按钮
    drawButton(btnStart);
    drawButton(btnStop);
    drawButton(btnUp);
    drawButton(btnDown);

    // 绘制状态start/run灯
    drawStatusBoxes();

    // 绘制底部文本框
    drawTextArea();
}

int main() {
    initgraph(g_width, g_height);
    initData();
    SetWorkingImage();
    setbkcolor(BLACK);
    cleardevice();

    // 双缓冲
    BeginBatchDraw();
    DWORD frame_start;

    while (!g_quit) {
        frame_start = GetTickCount();
        checkMouse();
        updateData();
        drawUI();
        EndBatchDraw();
        Sleep(5);
        BeginBatchDraw();
    }

    g_dataFile.close();
    g_logFile.close();
    closegraph();
    return 0;
}
