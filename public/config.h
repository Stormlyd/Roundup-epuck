#ifndef CONFIG_H
#define CONFIG_H

#include "language.h"
#include "pf.h"

//system config
#define REG_RUN                             "HKEY_CURRENT_USER\\Software\\Microsoft\\Windows\\CurrentVersion\\Run"
#define SCREEN_WIDTH                        1366                            //显示器宽度
#define SCREEN_HEIGHT                       880                             //显示器高度

#define ENABLE_MEMWATCH                     (0)                             //内存监控

#define DEBUG_MODE_SOFTWARA                 (false)                         //软件调试模式
#define INIT_ROBOT_NUMBER                   (0)                           //初始机器人个数
#define AGENT_DEBUG                         (false)                         //开启代理调试

#define MANAGER_PRINT                       0                               //管理器打印

//path config
const QString APP_PATH = pfAppPath();
const QString SQL_PATH = APP_PATH + "sql\\ZooidManager.db3";                //数据库文件
const QString CHARGE_PATH = APP_PATH + "json\\chargePosition.json";         //充电桩位置配置文件
const QString LOG_PATH = APP_PATH + "\\output\\";                           //日志文件路径
const QString MAIN_GIF_PATH = APP_PATH + "\\movie\\mainMode.gif";           //屏保演示gif
const QString FIGURE_MODE_GIF_PATH = APP_PATH + "\\movie\\figureMode.gif";  //自绘模式演示gif
const QString DRAW_MODE_GIF_PATH = APP_PATH + "\\movie\\drawMode.gif";      //自绘模式演示gif
const QString VIDEO_PATH = APP_PATH + "\\movie\\display.mp4";
//receiver config
#define NUM_ZOOIDS_PER_RECEIVER             10                              // 每个接收器的zooid数量

//algorithm config
#define ROBOT_DIAMETER                      0.080f                          // 机器人直径 60mm
#define ROBOT_RADIUS                        (ROBOT_DIAMETER / 2.0f)         // 机器人半径
#define MaxAccel                            0.0001f                         // 最大加速度
#define MaxSpeed                            0.018f   //0.03f                 //代理的最大速度 需要和实际机器人匹配
#define WheelTrack                          0.021f                          // 机器人的轮距
#define NeighborDist                        (ROBOT_DIAMETER * 1.5f)         // ORCA算法中的最大邻居距离
#define MaxNeighbors                        9                               // ORCA算法中的最大邻域数
#define GoalRadius                          (1.0f * ROBOT_RADIUS)           // 目标半径

#define ZOOID_RUN_SPEED                     100

//需要投影数据 计算
#define COORDINATES_MIN_X                   63.0f
#define COORDINATES_MAX_X                   960.0f
#define COORDINATES_MIN_Y                   229.0f
#define COORDINATES_MAX_Y                   795.0f
#define ROBOT_FIELD_MARGIN                  ROBOT_DIAMETER * 1.0f           // 机器人场边距

#define SYSTEM_UPDATE_FREQUENCY             60.0f                                         // 更新频率
#define SYSTEM_UPDATE_PERIOD				(1000.0f / SYSTEM_UPDATE_FREQUENCY)           // 更新周期
#define TIME_TO_ORIENTATION                 3.5f * (float)(SYSTEM_UPDATE_PERIOD)/1000.0f  // 3.5f * timeStep
#define ZOOID_WATCHDOG_TIMEOUT              5000                                          //ms

#define BATTERY_HIGH                        80      //电量在80%以上电量为绿色
#define BATTERY_LOW                         40      //电量在40%以上电量为黄色,下电量为红色

#define MAX_NB_ZOOIDS                       30      //最多Zooid数量  建议不超过500个，否则影响算法时间复杂度, 本系统使用30个
#define MAX_FOLLOW_ZOOID_COUNT              5       //跟随模式中的最多使用的机器人数
#define MAX_FOLLOW_MOVE_SPEED               0.5f    //指挥棒的最大移动速度m/s

const float PI = 3.141592653589793f;

//模拟器模式
enum SimulationMode {
    Off,            //关
    On,             //开
    NoPlanning      //无方案
};

//分配模式
enum AssignmentMode{
    OptimalAssignment = 1,  //最优分配
    NaiveAssignment = 0     //次优分配
};

//分配方案
enum PlanningMode{
    ChargePlanning = 0,     //充电方案
    TrianglePlanning,       //三角形
    ReactPlanning,          //矩形
    CrossPlanning,          //十字
    FivepointedPlanning,    //五角星
    HexagonPlanning,        //六边形
    RandomPlanning,         //位置随机
    CirculPlanning,         //圆形
    DrawpathPlanning,       //自绘路径
    FollowPlanning,         //跟随模式
    VoronoiPlanning,        //Voronoi覆盖控制模式
    NullPlanning,           //空闲
};

enum AppWidget{
    HomeWidget = 0,
    AdminWidget,
    MenuWidget,
};

#endif // CONFIG_H
