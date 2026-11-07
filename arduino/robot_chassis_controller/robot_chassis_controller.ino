/*
 * 零件分拣机器人系统 - 【视觉非阻塞通讯 + 模拟量突变泊车 + 正方体重试版】
 * 功能：自动循迹、视觉抓取、泊车入库
 *
 * 硬件平台：Arduino AVR 架构 (Uno / Nano / Mega / Basra 兼容)
 * 依赖库  ：ServoTimer2.h (利用 Timer2 规避与电机 PWM Timer1 的通道冲突)
 * 通信协议：115200 波特率，非阻塞行文本，格式 "class_id,cx,cy,pixel_ratio\n"
 */
#include <ServoTimer2.h> 

// ================= 引脚定义 =================
#define L_MOTOR_POS 9      // 左电机正转
#define L_MOTOR_NEG 10     // 左电机反转
#define R_MOTOR_POS 5      // 右电机正转
#define R_MOTOR_NEG 6      // 右电机反转

#define SENSOR_1 A3        // 最左侧循迹传感器
#define SENSOR_2 A4        // 中间左传感器
#define SENSOR_3 A0        // 中间右传感器
#define SENSOR_4 A2        // 最右侧循迹传感器

#define BASE_PIN 7         // 底座舵机
#define SHOULDER_PIN 11    // 肩部舵机
#define ELBOW_PIN 12       // 肘部舵机
#define CLAW_PIN 8         // 手爪舵机
const int IR_PIN = 2;     // 侧方红外

ServoTimer2 baseS, shoulderS, elbowS, clawS;
#define MAP_270(deg) (int)((deg) / 1.5)   // 将物理角度(0~270)映射为舵机库输入值(0~180)

// ================= 底盘校准 =================
#define BASE_SPEED_L 128       // 左轮基础速度
#define BASE_SPEED_R 128      // 右轮基础速度
#define MAX_SPEED 240          // 最大速度限制
#define MIN_SPEED -230         // 最小速度限制（最大后退）

// 电机最低启动PWM阈值(死区补偿)
#define DEADZONE_PWM 115

const float LEFT_COMPENSATION = 1.00;   // 左轮补偿系数
const float RIGHT_COMPENSATION = 1.00;  // 右轮补偿系数

// ================= 时间参数 =================
#define IGNORE_CORNER_TIME 35000   // 发车后忽略直角弯的时间(ms)
#define MARKER_PROTECT_TIME 5000   // 黑线标记保护时间
#define INITIAL_DASH_TIME 3400     // 爆发直冲总时间


// 第二次转弯相关参数（保留宏定义以防它处引用，实际逻辑已重写）
#define REVERSE_TRACK_SPEED  125     // 后退循迹基础速度
#define PARK_TRACKING_MIN_TIME 1500  // 泊车最小前进时间
#define REVERSE_BLIND_TIME   3600    // 后退盲退时间

int cornerLeftDebounce = 0;   // 直角弯左转防抖计数

// ================= PID 参数 =================
float Kp = 2.0, Ki = 0.0, Kd = 10.0;               // PID系数
float error = 0, lastError = 0, integral = 0, derivative = 0, pidValue = 0;

// ================= 机械臂姿态参数 =================
const int PHY_CLAW_RELEASE = 128;   // 手爪释放角度
const int PHY_READY_CLAW   = 150;   // 准备抓取时手爪张开角度

const int PHY_CLAW_CLOSE_CUBE     = 111; // 夹紧立方体
const int PHY_CLAW_CLOSE_CYLINDER = 109; // 夹紧圆柱体
const int PHY_CLAW_CLOSE_PRISM    = 108; // 夹紧棱柱体

const int PHY_READY_BASE = 36;      // 底座准备角度
const int PHY_READY_SHOULDER = 120; // 肩部准备角度
const int PHY_READY_ELBOW = 189;    // 肘部准备角度

// 各物体放置位置角度
const int PLACE_BASE_CUBE     = 217; const int PLACE_SHOULDER_CUBE     = 95;  const int PLACE_ELBOW_CUBE     = 233; 
const int PLACE_BASE_CYLINDER = 177; const int PLACE_SHOULDER_CYLINDER = 104; const int PLACE_ELBOW_CYLINDER = 224; 
const int PLACE_BASE_PRISM    = 258; const int PLACE_SHOULDER_PRISM    = 104; const int PLACE_ELBOW_PRISM    = 225; 

// 各物体取件位置角度
const int RETRIEVE_BASE_CUBE     = 218; const int RETRIEVE_SHOULDER_CUBE     = 95;  const int RETRIEVE_ELBOW_CUBE     = 234; 
const int RETRIEVE_BASE_CYLINDER = 176; const int RETRIEVE_SHOULDER_CYLINDER = 104; const int RETRIEVE_ELBOW_CYLINDER = 225; 
const int RETRIEVE_BASE_PRISM    = 258; const int RETRIEVE_SHOULDER_PRISM    = 104; const int RETRIEVE_ELBOW_PRISM    = 225;

// 取件后提升角度
const int RETRIEVE_LIFT_SHOULDER_CUBE     = 74; const int RETRIEVE_LIFT_ELBOW_CUBE     = 236; 
const int RETRIEVE_LIFT_SHOULDER_CYLINDER = 77; const int RETRIEVE_LIFT_ELBOW_CYLINDER = 226; 
const int RETRIEVE_LIFT_SHOULDER_PRISM    = 77; const int RETRIEVE_LIFT_ELBOW_PRISM    = 226; 

const int RETRIEVE_CARRY_SHOULDER = 120; const int RETRIEVE_CARRY_ELBOW = 174; // 搬运姿态
const int RETURN_BASE = 157; const int RETURN_SHOULDER = 182; const int RETURN_ELBOW = 122; // 回归姿态

const int FINISH_BASE = 36; const int FINISH_SHOULDER = 70; const int FINISH_ELBOW = 243; const int FINISH_CLAW = 123; // 完成姿态

int tarBase = MAP_270(PHY_READY_BASE), curBase = MAP_270(PHY_READY_BASE); 
int tarShoulder = MAP_270(PHY_READY_SHOULDER), curShoulder = MAP_270(PHY_READY_SHOULDER);  
int tarElbow = MAP_270(PHY_READY_ELBOW), curElbow = MAP_270(PHY_READY_ELBOW);
int tarClaw = MAP_270(PHY_READY_CLAW), curClaw = MAP_270(PHY_READY_CLAW); 

// ================= 视觉抓取变量 =================
const int CENTER_X = 320, DEADZONE_X = 45;     // 图像中心X及死区
const int CENTER_Y = 240, DEADZONE_Y = 40;     // 图像中心Y及死区
float RATIO_CUBE = 10.8;       // 立方体距离比值
float RATIO_CYLINDER = 12.2;   // 圆柱体距离比值
float RATIO_PRISM = 10.8;       // 棱柱体距离比值

const float DEADZONE_RATIO = 1.5;   // 距离死区
const int SPEED_DELAY = 20;         // 舵机平滑移动延时

int aligned_count = 0;        // 对准计数
bool hasObject = false;       // 是否持有物体
int visual_grab_count = 0;    // 已抓取次数
int retrieve_stage = 0;       // 投放阶段

int ir_detect_count = 0;      // 红外检测计数
int ir_clear_count = 0;       // 红外离开计数
unsigned long deliveryClearTime = 0; 
bool waiting_for_clear = false; // 等待红外离开
int park_ir_count = 0;        // 泊车红外计数

int last_grabbed_class_id = -1;      // 上次抓取的物体类别
unsigned long verifyStartTime = 0;   // 验证开始时间
unsigned long firstGrabCompleteTime = 0; // 首次抓取完成时间
unsigned long conditionStartTime = 0;    // 条件满足计时

String serialBuffer = "";     // 串口接收缓冲区
String latestMsg = "";        // 最新完整消息
bool newMsgReady = false;     // 新消息标志

// 正方体重试专用变量
bool cubeRetryMode = false;          // 是否处于正方体重试
float RATIO_CUBE_RETRY = 16.6;       // 重试时的目标RATIO
const float DEADZONE_RATIO_RETRY = 2.3; // 重试时的死区

// 后退离开垫子阶段变量
bool backAwayActive = false;         // 是否处于后退离开垫子模式
int backAwayClearCount = 0;          // 连续无垫子计数

// ================= 状态机定义 =================
#define STATE_VISION_GRAB      0   // 视觉抓取
#define STATE_VISION_VERIFY    1   // 抓取验证
#define STATE_TRACKING         2   // 循迹
#define STATE_BLACK_BURST      3   // 黑线爆发
#define STATE_DELIVERY_HUNT    4   // (保留)
#define STATE_DELIVERING       5   // 投放
#define STATE_PARK_FORWARD     6   // (保留)
#define STATE_PARK_TURN1       7   // (保留)
#define STATE_PARK_TRACKING    8   // 泊车循迹
#define STATE_PARK_TURN2       9   // 第二次转弯
#define STATE_PARK_ALIGN_FWD   10  // 前向校准
#define STATE_PARK_REVERSE_TRACKING 11 // 后退循迹
#define STATE_FINISHED         12  // 完成

#define STATE_WAITING          99  // 非阻塞等待
#define STATE_CORNER_CONFIRM   13  // 直角弯确认
#define STATE_CORNER_LEFT     14  // 直角弯左转
#define STATE_PARK_BACK_EXTRA 15  // 离开垫子后额外后退0.4秒

int runState = STATE_VISION_GRAB; 
unsigned long trackStartTime = 0;   
unsigned long burstStartTime = 0;   
unsigned long parkStartTime = 0;    
unsigned long corner_confirm_start = 0;   
unsigned long parkTrackingStart = 0;

unsigned long waitStartTime = 0;
unsigned long waitDuration = 0;
int nextStateAfterWait = STATE_FINISHED;

// 第二次转弯状态标志（包含 0~4 五个精细阶段）
int turn2Phase = 0; 
bool turn2SeekWhite = false; 

// ================= 函数声明 =================
void motorControl(int leftSpeed, int rightSpeed);
float calcDigitalError(int s[4], bool allWhite);
float calcDigitalErrorPark(int s[4], bool allWhite); 
void performPIDTracking(int s[4], bool allWhite, int baseL, int baseR);
void performPIDTrackingPark(int s[4], bool allWhite, int baseL, int baseR); 
void performPIDReverse(int s[4], bool allWhite, int reverseSpeed);
void servoWriteDeg(ServoTimer2& servo, int degree);
void processTracking(int class_id, int cx, int cy, float ratio);
void executeSmoothGrab(int target_class_id);
void executeRetrieve(int target_class_id);
void UpdateServoSmooth();
void WaitUntilReach();

void resetPID() {
  error = 0; lastError = 0; integral = 0; derivative = 0; pidValue = 0;
}

void flushVisionData() {
  while(Serial.available() > 0) { Serial.read(); }
  serialBuffer = "";
  newMsgReady = false;
}

void initiateWait(unsigned long duration, int nextState) {
    motorControl(0, 0);               
    waitStartTime = millis();         
    waitDuration = duration;          
    nextStateAfterWait = nextState;   
    runState = STATE_WAITING;         
}

void setup() {
  Serial.begin(115200); 
  pinMode(L_MOTOR_POS, OUTPUT); pinMode(L_MOTOR_NEG, OUTPUT);
  pinMode(R_MOTOR_POS, OUTPUT); pinMode(R_MOTOR_NEG, OUTPUT);
  pinMode(SENSOR_1, INPUT); pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT); pinMode(SENSOR_4, INPUT);
  pinMode(IR_PIN, INPUT);
  TCCR1A = (TCCR1A & 0b11111100) | 0b00000001; 
  TCCR1B = (TCCR1B & 0b11100000) | 0b00001011; 
  baseS.attach(BASE_PIN); shoulderS.attach(SHOULDER_PIN);
  elbowS.attach(ELBOW_PIN); clawS.attach(CLAW_PIN);
  tarBase = MAP_270(PHY_READY_BASE); 
  tarShoulder = MAP_270(PHY_READY_SHOULDER);
  tarElbow = MAP_270(PHY_READY_ELBOW); 
  tarClaw = MAP_270(PHY_READY_CLAW);
  WaitUntilReach(); 
  Serial.println("✅ 5步完美转弯法版 就绪！");
  runState = STATE_VISION_GRAB;
}

void loop() {
  if (runState == STATE_FINISHED) {
    motorControl(0, 0); 
    return; 
  }
  UpdateServoSmooth();
  
  // 非阻塞串口接收
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      latestMsg = serialBuffer;   
      serialBuffer = "";          
      newMsgReady = true;         
    } else if (inChar != '\r') {
      serialBuffer += inChar;     
    }
  }
  
  int s[4];
  s[0] = (digitalRead(SENSOR_1) == HIGH) ? 1 : 0;
  s[1] = (digitalRead(SENSOR_2) == HIGH) ? 1 : 0;
  s[2] = (digitalRead(SENSOR_3) == HIGH) ? 1 : 0;
  s[3] = (digitalRead(SENSOR_4) == HIGH) ? 1 : 0;
  int activeCount = s[0] + s[1] + s[2] + s[3];
  bool allWhite = (activeCount == 0);
  
  switch (runState) {
    case STATE_WAITING:
      if (millis() - waitStartTime >= waitDuration) {
         resetPID(); 
         if (nextStateAfterWait == STATE_PARK_TRACKING) parkTrackingStart = millis();
         if (nextStateAfterWait == STATE_PARK_TURN2 || nextStateAfterWait == STATE_PARK_ALIGN_FWD || nextStateAfterWait == STATE_PARK_REVERSE_TRACKING) parkStartTime = millis();
         if (nextStateAfterWait == STATE_PARK_TURN1) parkStartTime = millis();
         
         // 特殊处理：如果即将进入前向校准，先给一个启动脉冲，防止PID输出过小转不动
         if (nextStateAfterWait == STATE_PARK_ALIGN_FWD) {
            motorControl(150, 140);
            delay(30);
            // 脉冲结束后立即刹车，接下来由 STATE_PARK_ALIGN_FWD 接管
            motorControl(0, 0);
         }

         // 进入第二次转弯前重置阶段状态
         if (nextStateAfterWait == STATE_PARK_TURN2) {
           turn2Phase = 0;
           turn2SeekWhite = false; 
         }
         runState = nextStateAfterWait;
      }
      break;

    case STATE_VISION_GRAB:
      motorControl(0, 0); 
      if (visual_grab_count >= 1 && millis() - firstGrabCompleteTime >= 40000) {
        Serial.println("⏰ 警告：距抓完第1个已超40秒！强行发车！");
        tarBase = MAP_270(FINISH_BASE); WaitUntilReach();
        tarShoulder = MAP_270(FINISH_SHOULDER); tarElbow = MAP_270(FINISH_ELBOW); tarClaw = MAP_270(FINISH_CLAW); WaitUntilReach();
        flushVisionData(); 
        resetPID(); 
        trackStartTime = millis(); 
        runState = STATE_TRACKING;
        break;
      }
      if (newMsgReady) {
        newMsgReady = false; 
        if (!hasObject && visual_grab_count < 3) {
          String dataStr = latestMsg;
          dataStr.trim();
          int c1 = dataStr.indexOf(','); int c2 = dataStr.indexOf(',', c1 + 1); int c3 = dataStr.indexOf(',', c2 + 1);
          if (c1 > 0 && c2 > 0 && c3 > 0) {
            int class_id = dataStr.substring(0, c1).toInt(); 
            if (class_id <= 2) { 
              int cx = dataStr.substring(c1 + 1, c2).toInt();
              int cy = dataStr.substring(c2 + 1, c3).toInt(); 
              float ratio = dataStr.substring(c3 + 1).toFloat(); 
              if (curBase == tarBase && curShoulder == tarShoulder && curElbow == tarElbow) {
                processTracking(class_id, cx, cy, ratio); 
              }
            }
          }
        }
      }
      break;

    case STATE_VISION_VERIFY:
      motorControl(0, 0);
      {
        bool still_see_it = false;
        if (millis() - verifyStartTime < 800) {
          if (newMsgReady) {
            newMsgReady = false; 
            String dataStr = latestMsg;
            dataStr.trim(); 
            int c1 = dataStr.indexOf(',');
            if (c1 > 0) {
              int class_id = dataStr.substring(0, c1).toInt();
              if (class_id == last_grabbed_class_id) { still_see_it = true; }
            }
          }
        } 
        if (still_see_it) {
          Serial.println("❌ 漏抓！调深阈值重试！");
          if (last_grabbed_class_id == 0) {
            cubeRetryMode = true;   
            tarShoulder = MAP_270(PHY_READY_SHOULDER);
            tarElbow = MAP_270(PHY_READY_ELBOW);
            Serial.println("🔧 正方体重试：RATIO=16.6，死区放大，强制重新对准");
          }
          hasObject = false;
          flushVisionData(); 
          runState = STATE_VISION_GRAB; 
        } 
        else if (millis() - verifyStartTime >= 800) {
          Serial.println("✅ 确认抓取成功！");
          visual_grab_count++; 
          cubeRetryMode = false;   
          if (visual_grab_count == 1) {
            firstGrabCompleteTime = millis();
            Serial.println("⏱️ 抓完第一个，40秒强制发车计时开始！");
          }
          if (visual_grab_count >= 3) {
            Serial.println("🚀 抓满3个！进入强力循迹！");
            tarBase = MAP_270(FINISH_BASE); WaitUntilReach();
            tarShoulder = MAP_270(FINISH_SHOULDER); tarElbow = MAP_270(FINISH_ELBOW); tarClaw = MAP_270(FINISH_CLAW); WaitUntilReach();
            flushVisionData();
            resetPID(); 
            trackStartTime = millis(); 
            runState = STATE_TRACKING;
          } else {
            hasObject = false;
            flushVisionData();
            runState = STATE_VISION_GRAB; 
          }
        }
      }
      break;

    case STATE_TRACKING:
      if (activeCount >= 3 && (millis() - trackStartTime < MARKER_PROTECT_TIME)) {
        runState = STATE_BLACK_BURST;
        burstStartTime = millis();
      } 
      else if (retrieve_stage < 3) {
        if (waiting_for_clear) {
          performPIDTracking(s, allWhite, BASE_SPEED_L, BASE_SPEED_R); 
          if (digitalRead(IR_PIN) == HIGH) { 
            ir_clear_count++;
            if (ir_clear_count >= 20 && (millis() - deliveryClearTime > 800)) { 
              waiting_for_clear = false; 
              ir_clear_count = 0; 
            }
          } else { 
            ir_clear_count = 0;
            deliveryClearTime = millis(); 
          }
        } else {
          if (digitalRead(IR_PIN) == LOW && (millis() - trackStartTime > 9000)) {
            ir_detect_count++;
            if (ir_detect_count >= 5) { 
              Serial.println("📦 锁定收纳盒！开始投放");
              ir_detect_count = 0;
              initiateWait(60, STATE_DELIVERING); 
            } else { performPIDTracking(s, allWhite, BASE_SPEED_L, BASE_SPEED_R); }
          } else { 
            ir_detect_count = 0; 
            performPIDTracking(s, allWhite, BASE_SPEED_L, BASE_SPEED_R); 
          }
        }
      }
      else {
        bool corner_seen = (s[0] == 1 && s[1] == 1) || (s[0] == 1 && s[2] == 1);
        if (corner_seen) {
          Serial.println("🏁 扫到直角标志！进行防误判确认...");
          resetPID();
          motorControl(0, 0);
          corner_confirm_start = millis();
          runState = STATE_CORNER_CONFIRM;
        }
        else {
          performPIDTracking(s, allWhite, BASE_SPEED_L, BASE_SPEED_R);
        }
      }
      break;

    case STATE_BLACK_BURST:
      {
        unsigned long burstElapsed = millis() - burstStartTime;
        if (burstElapsed < 1500) motorControl(235, 240); 
        else if (burstElapsed < INITIAL_DASH_TIME) motorControl(165, 165);
        else { resetPID(); runState = STATE_TRACKING; }
      }
      break;

    case STATE_DELIVERING:
      {
        int target_class;
        if (retrieve_stage == 0) target_class = 2;      
        else if (retrieve_stage == 1) target_class = 1; 
        else target_class = 0;                          
        executeRetrieve(target_class); 
        retrieve_stage++;
        if (retrieve_stage >= 3) Serial.println("✅ 3次投放完毕！前往直角弯！");
        else Serial.println("➡️ 投放完毕！起步循迹...");
        waiting_for_clear = true; ir_clear_count = 0; 
        deliveryClearTime = millis(); 
        flushVisionData();
        resetPID(); 
        runState = STATE_TRACKING;
      }
      break;

  case STATE_CORNER_CONFIRM:
    motorControl(122, 122);  
    if (s[0] == 0 && s[1] == 0 && s[2] == 0 && s[3] == 0) {
     Serial.println("✅ 确认是真直角弯(已丢线)！开始左转...");
      cornerLeftDebounce = 0;   // 重置防抖计数
      runState = STATE_CORNER_LEFT;
    }
    break;

  case STATE_CORNER_LEFT:
      motorControl(-190, 185);  
      if (s[1] == 1 || s[2] == 1) {
          cornerLeftDebounce++;                // 满足条件就累加
          if (cornerLeftDebounce >= 3) {       // 连续 3 次才算真正回正
             Serial.println("🎯 车头已回正！切入强力泊车循迹...");
              // 进入循迹前的短时左转摆正
              motorControl(-10, 220); 
              delay(450);
              motorControl(180, 180); 
              delay(200);
             conditionStartTime = 0;
              park_ir_count = 0;
             parkTrackingStart = millis();
              resetPID();
              runState = STATE_PARK_TRACKING;
              cornerLeftDebounce = 0;          // 清零，为下一次准备
          }
      } else {
          cornerLeftDebounce = 0;              // 不满足立即清零，必须连续压线
      }
      break;

    case STATE_PARK_TRACKING:
      {
        if (backAwayActive) {
          motorControl(-125, -125);          
          if (digitalRead(IR_PIN) == HIGH) { 
            backAwayClearCount++;
            if (backAwayClearCount >= 2) {
              backAwayActive = false;
              backAwayClearCount = 0;
              park_ir_count = 0;          
              waitStartTime = millis();   
              runState = STATE_PARK_BACK_EXTRA; 
            }
          } else {
            backAwayClearCount = 0;        
          }
          break; 
        }

        if (millis() - parkTrackingStart < PARK_TRACKING_MIN_TIME) {
          if (allWhite) { motorControl(130, 130); } 
          else { performPIDTrackingPark(s, allWhite, BASE_SPEED_L, BASE_SPEED_R); };
          break;
        }
        
        performPIDTrackingPark(s, allWhite, BASE_SPEED_L, BASE_SPEED_R);
        
        if (digitalRead(IR_PIN) == LOW) {
          park_ir_count++;
          if (park_ir_count >= 3) {
            Serial.println("检测到垫子3次，开始后退离开...");
            backAwayActive = true;
            backAwayClearCount = 0;
            park_ir_count = 0;  
          }
        } else {
          park_ir_count = 0;
        }
      }
      break;

    case STATE_PARK_BACK_EXTRA:
      motorControl(-127, -122);
      if (millis() - waitStartTime >= 300) {
        parkStartTime = millis();  
        turn2Phase = 0;            
        turn2SeekWhite = false;    
        runState = STATE_PARK_TURN2;
      }
      break;

    // ================= 第二次转弯 (终极融合五步法) =================
    case STATE_PARK_TURN2:
      {
        unsigned long turnElapsed = millis() - parkStartTime;
        
        if (turn2Phase == 0) {
          // 阶段0：以 motorControl(-155, 170) 旋转，刚执行1.0秒内不进行全白识别
          motorControl(-130, 200);
          
          if (turnElapsed >= 1000 && allWhite) {
            motorControl(130, -200); // 短暂反向制动
            delay(30);               
            motorControl(0, 0);      
            Serial.println("🎯 四个传感器全部识别为全白，切入阶段 1");
            turn2Phase = 1;
            parkStartTime = millis(); // 为阶段 1 重新计时
          }
        } 
        else if (turn2Phase == 1) {
          // 阶段1：执行 motorControl(170, -155)，直到右侧任意传感器扫到黑线
          motorControl(-110, -180);
          
          if (s[2] == 1 || s[3] == 1) {
            motorControl(-170, 180); // 短暂反向制动
            delay(50);
            motorControl(0, 0);      
            Serial.println("🎯 右侧传感器扫到黑线，切入阶段 2");
            turn2Phase = 2;
            parkStartTime = millis(); 
          }
        } 
        else if (turn2Phase == 2) {
          // 阶段2：执行 motorControl(112, 112) 前进，最多 5 秒
          motorControl(122, 122);
          
          // 如果左侧传感器任何一个识别到黑线，或者 5 秒超时
          if (s[0] == 1 || s[1] == 1 || turnElapsed >= 5000) {
            motorControl(-124, -124); // 短暂反向制动
            delay(100);
            motorControl(0, 0);       
            Serial.println("✅ 左侧碰到黑线或前进5秒超时！切入阶段 3 (前进找全白)");
            turn2Phase = 3;           
            parkStartTime = millis();
          }
        }
        else if (turn2Phase == 3) {
          // 阶段3：(你原本的阶段2) 前进，如果四个传感器都检测到为全白，刹车停住
          motorControl(122, 122);
          
          if (allWhite) {
            motorControl(-125, -125); // 短暂反向制动
            delay(100);
            motorControl(0, 0); 
            Serial.println("🎯 前进找到全白，切入阶段 4 (左转找右线)");
            turn2Phase = 4;     
            parkStartTime = millis();
          }
        }
        else if (turn2Phase == 4) {
          // 阶段4：(你原本的阶段3) 一直左转，直到右侧两个传感器任何一个检测到黑线，刹车停住
          motorControl(-175, 180);
          
          bool rightLineSeen = (s[2] == 1 || s[3] == 1);
          if (rightLineSeen) {
            motorControl(170, -160); // 短暂反转制动
            delay(100);
            motorControl(0, 0); // 刹车停住
            turn2Phase = 0; // 重置阶段，为下一次循环准备
            Serial.println("🎯 右侧再次扫到黑线，姿态已完美回正！进入前向校准...");
            initiateWait(150, STATE_PARK_ALIGN_FWD);
            // 启动脉冲已移到 STATE_WAITING 中执行，此处不再重复
          }
        }
      }
      break;

    case STATE_PARK_ALIGN_FWD:
      if (millis() - parkStartTime < 2200) {  
        performPIDTrackingPark(s, allWhite, BASE_SPEED_L, BASE_SPEED_R);
      } else {
        Serial.println("✅ 前进校准完成！开始后退盲退...");
        conditionStartTime = 0; 
        initiateWait(150, STATE_PARK_REVERSE_TRACKING);
      }
      break;

    case STATE_PARK_REVERSE_TRACKING:
      {
        if (millis() - parkStartTime < REVERSE_BLIND_TIME) {
          motorControl(-118, -118); //之前是115和120
        } else {
          Serial.println("✅ 后退盲退完成，泊车停驻！复位系统！");
          motorControl(0, 0);
          
          visual_grab_count = 0;
          retrieve_stage = 0;
          hasObject = false;
          firstGrabCompleteTime = 0; 
          cubeRetryMode = false;   
          backAwayActive = false;
          backAwayClearCount = 0;
          
          tarBase = MAP_270(PHY_READY_BASE); 
          tarShoulder = MAP_270(PHY_READY_SHOULDER);
          tarElbow = MAP_270(PHY_READY_ELBOW); 
          tarClaw = MAP_270(PHY_READY_CLAW);
          WaitUntilReach(); 
          
          flushVisionData(); 
          runState = STATE_VISION_GRAB; 
        }
      }
      break;
  }
}

// ================= 误差计算 =================
float calcDigitalError(int s[4], bool allWhite) {
  if (allWhite) return 0.0; 
  float currentError = 0.0;
  if      (s[0]==1 && s[1]==0 && s[2]==0 && s[3]==0) currentError = -10.0; 
  else if (s[0]==1 && s[1]==1 && s[2]==0 && s[3]==0) currentError = -6.0; 
  else if (s[0]==0 && s[1]==1 && s[2]==0 && s[3]==0) currentError = -3.0; 
  else if (s[0]==0 && s[1]==1 && s[2]==1 && s[3]==0) currentError = 0.0;  
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==0) currentError = 3.0;  
  else if (s[0]==0 && s[1]==0 && s[2]==1 && s[3]==1) currentError = 6.0;  
  else if (s[0]==0 && s[1]==0 && s[2]==0 && s[3]==1) currentError = 10.0;  
  else if (s[0]==1 && s[3]==0) currentError = -10.0;
  else if (s[3]==1 && s[0]==0) currentError = 10.0;
  else { currentError = lastError; } 
  return currentError;
}

float calcDigitalErrorPark(int s[4], bool allWhite) {
  if (allWhite) return 0.0; 
  float currentError = 0.0;
  if (s[1] == 1 && s[2] == 1) currentError = 0.0; 
  else if (s[0] == 1) { if (s[1] == 1) currentError = -6.0; else currentError = -11.0; }
  else if (s[3] == 1) { if (s[2] == 1) currentError = 6.0; else currentError = 11.0; }
  else if (s[1] == 1 && s[2] == 0) currentError = -4.0; 
  else if (s[1] == 0 && s[2] == 1) currentError = 4.0;  
  else currentError = lastError; 
  return currentError;
}

// ================= PID 控制 =================
void performPIDTracking(int s[4], bool allWhite, int baseL, int baseR) {
  error = calcDigitalError(s, allWhite);
  integral += error; integral = constrain(integral, -30, 30);
  derivative = error - lastError;
  float current_Kp = Kp; float current_Kd = Kd;
  if (abs(error) <= 3.0) { current_Kp = Kp * 0.7; current_Kd = Kd * 0.5; } 
  else if (abs(error) >= 8.0) { current_Kp = Kp * 2.0; current_Kd = Kd * 1.5; }
  pidValue = current_Kp * error + Ki * integral + current_Kd * derivative;
  lastError = error;
  int turnPower = (int)(pidValue * 22); turnPower = constrain(turnPower, -180, 240);
  int leftSpeed = constrain((baseL * LEFT_COMPENSATION) + turnPower, MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((baseR * RIGHT_COMPENSATION) - turnPower, MIN_SPEED, MAX_SPEED);
  motorControl(leftSpeed, rightSpeed);
}

void performPIDTrackingPark(int s[4], bool allWhite, int baseL, int baseR) {
  error = calcDigitalErrorPark(s, allWhite);
  integral += error; integral = constrain(integral, -30, 30);
  derivative = error - lastError;
  float park_Kp = 2.8; float park_Kd = 14.0; 
  if (abs(error) <= 4.0) { park_Kp = 2.0; park_Kd = 10.0; } 
  pidValue = park_Kp * error + Ki * integral + park_Kd * derivative;
  lastError = error;
  int turnPower = (int)(pidValue * 22); turnPower = constrain(turnPower, -220, 255);
  int leftSpeed = constrain((baseL * LEFT_COMPENSATION) + turnPower, MIN_SPEED, MAX_SPEED);
  int rightSpeed = constrain((baseR * RIGHT_COMPENSATION) - turnPower, MIN_SPEED, MAX_SPEED);
  motorControl(leftSpeed, rightSpeed);
}

void performPIDReverse(int s[4], bool allWhite, int reverseSpeed) {
  error = calcDigitalError(s, allWhite);
  integral += error; integral = constrain(integral, -30, 30);
  derivative = error - lastError;
  pidValue = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  int turnPower = (int)(pidValue * 10); turnPower = constrain(turnPower, -reverseSpeed / 2, reverseSpeed / 2);
  int leftSpeed  = -reverseSpeed - turnPower; int rightSpeed = -reverseSpeed + turnPower;
  leftSpeed = constrain(leftSpeed, MIN_SPEED, -20); rightSpeed = constrain(rightSpeed, MIN_SPEED, -20);
  motorControl(leftSpeed, rightSpeed);
}

// ================= 电机与舵机驱动 =================
void servoWriteDeg(ServoTimer2& servo, int degree) {
  int pulseWidth = map(degree, 0, 180, 500, 2500);
  servo.write(pulseWidth);
}

void motorControl(int leftSpeed, int rightSpeed) {
  // 核心死区补偿逻辑
  if (leftSpeed > 0 && leftSpeed < DEADZONE_PWM) leftSpeed = DEADZONE_PWM;
  if (leftSpeed < 0 && leftSpeed > -DEADZONE_PWM) leftSpeed = -DEADZONE_PWM;
  if (rightSpeed > 0 && rightSpeed < DEADZONE_PWM) rightSpeed = DEADZONE_PWM;
  if (rightSpeed < 0 && rightSpeed > -DEADZONE_PWM) rightSpeed = -DEADZONE_PWM;

  if (leftSpeed > 0) { analogWrite(L_MOTOR_POS, leftSpeed); analogWrite(L_MOTOR_NEG, 0); } 
  else if (leftSpeed < 0) { analogWrite(L_MOTOR_POS, 0); analogWrite(L_MOTOR_NEG, -leftSpeed); } 
  else { analogWrite(L_MOTOR_POS, 0); analogWrite(L_MOTOR_NEG, 0); }
  if (rightSpeed > 0) { analogWrite(R_MOTOR_POS, rightSpeed); analogWrite(R_MOTOR_NEG, 0); } 
  else if (rightSpeed < 0) { analogWrite(R_MOTOR_POS, 0); analogWrite(R_MOTOR_NEG, -rightSpeed); } 
  else { analogWrite(R_MOTOR_POS, 0); analogWrite(R_MOTOR_NEG, 0); }
}

// ================= 视觉追踪与抓取 =================
void processTracking(int class_id, int cx, int cy, float ratio) {
  bool x_ok = false, y_ok = false, dist_ok = false;
  float current_target_ratio = RATIO_CUBE; 
  if (class_id == 1) current_target_ratio = RATIO_CYLINDER;   
  else if (class_id == 2) current_target_ratio = RATIO_PRISM;      
  
  if (class_id == 0 && cubeRetryMode) {
    current_target_ratio = RATIO_CUBE_RETRY;
  }
  
  if (cx > CENTER_X + DEADZONE_X) tarBase -= 1;
  else if (cx < CENTER_X - DEADZONE_X) tarBase += 1;
  else x_ok = true;
  
  float active_deadzone = (class_id == 0 && cubeRetryMode) ? DEADZONE_RATIO_RETRY : DEADZONE_RATIO;
  
  if (ratio < current_target_ratio - active_deadzone) tarShoulder += 1; 
  else if (ratio > current_target_ratio + active_deadzone) tarShoulder -= 1; 
  else dist_ok = true; 
  
  if (cy < CENTER_Y - DEADZONE_Y) tarElbow -= 1; 
  else if (cy > CENTER_Y + DEADZONE_Y) tarElbow += 1; 
  else y_ok = true;
  
  tarBase = constrain(tarBase, MAP_270(5), MAP_270(260));
  tarShoulder = constrain(tarShoulder, MAP_270(30), MAP_270(250)); 
  tarElbow = constrain(tarElbow, MAP_270(30), MAP_270(250)); 
  
  if (tarShoulder == MAP_270(250) || tarShoulder == MAP_270(30)) {
    dist_ok = true;
  }
  if (tarElbow == MAP_270(250) || tarElbow == MAP_270(30)) {
    y_ok = true;
  }
  if (tarBase == MAP_270(260) || tarBase == MAP_270(5)) {
    x_ok = true;
  }
  if (x_ok && y_ok && dist_ok) {
    aligned_count++;
    if (aligned_count >= 5) { 
      hasObject = true;      
      executeSmoothGrab(class_id);    
      aligned_count = 0; 
    }
  } else {
    aligned_count = 0; 
  }
}

void executeSmoothGrab(int target_class_id) {
  int current_claw_close = PHY_CLAW_CLOSE_CUBE; 
  int target_base = PLACE_BASE_CUBE;
  int target_shoulder = PLACE_SHOULDER_CUBE;
  int target_elbow = PLACE_ELBOW_CUBE;
  int current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_CUBE;
  int current_lift_elbow = RETRIEVE_LIFT_ELBOW_CUBE;
  
  if (target_class_id == 0) {
    current_claw_close = PHY_CLAW_CLOSE_CUBE; target_base = PLACE_BASE_CUBE; target_shoulder = PLACE_SHOULDER_CUBE; target_elbow = PLACE_ELBOW_CUBE;
    current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_CUBE; current_lift_elbow = RETRIEVE_LIFT_ELBOW_CUBE;
  } else if (target_class_id == 1) {
    current_claw_close = PHY_CLAW_CLOSE_CYLINDER; target_base = PLACE_BASE_CYLINDER; target_shoulder = PLACE_SHOULDER_CYLINDER; target_elbow = PLACE_ELBOW_CYLINDER;
    current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_CYLINDER; current_lift_elbow = RETRIEVE_LIFT_ELBOW_CYLINDER;
  } else if (target_class_id == 2) {
    current_claw_close = PHY_CLAW_CLOSE_PRISM; target_base = PLACE_BASE_PRISM; target_shoulder = PLACE_SHOULDER_PRISM; target_elbow = PLACE_ELBOW_PRISM;
    current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_PRISM; current_lift_elbow = RETRIEVE_LIFT_ELBOW_PRISM;
  }
  
  tarClaw = MAP_270(current_claw_close); WaitUntilReach(); delay(500); 
  tarShoulder = MAP_270(PHY_READY_SHOULDER); tarElbow = MAP_270(PHY_READY_ELBOW); WaitUntilReach();
  tarBase = MAP_270(target_base); WaitUntilReach();
  tarShoulder = MAP_270(current_lift_shoulder); tarElbow = MAP_270(current_lift_elbow); WaitUntilReach();
  tarShoulder = MAP_270(target_shoulder); tarElbow = MAP_270(target_elbow); WaitUntilReach();delay(500);
  tarClaw = MAP_270(PHY_CLAW_RELEASE); WaitUntilReach(); delay(300);
  tarShoulder = MAP_270(PHY_READY_SHOULDER); tarElbow = MAP_270(PHY_READY_ELBOW); tarClaw = MAP_270(133); WaitUntilReach();delay(300);
  tarBase = MAP_270(PHY_READY_BASE); tarClaw = MAP_270(PHY_READY_CLAW);WaitUntilReach();
  
  last_grabbed_class_id = target_class_id;
  verifyStartTime = millis();
  flushVisionData(); 
  runState = STATE_VISION_VERIFY;
}

void executeRetrieve(int target_class_id) {
  int current_claw_close = PHY_CLAW_CLOSE_CUBE; 
  int target_base = RETRIEVE_BASE_CUBE; int target_shoulder = RETRIEVE_SHOULDER_CUBE; int target_elbow = RETRIEVE_ELBOW_CUBE;
  int current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_CUBE; int current_lift_elbow = RETRIEVE_LIFT_ELBOW_CUBE;
  
  if (target_class_id == 1) {
    current_claw_close = PHY_CLAW_CLOSE_CYLINDER; target_base = RETRIEVE_BASE_CYLINDER; target_shoulder = RETRIEVE_SHOULDER_CYLINDER; target_elbow = RETRIEVE_ELBOW_CYLINDER;
    current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_CYLINDER; current_lift_elbow = RETRIEVE_LIFT_ELBOW_CYLINDER;
  } else if (target_class_id == 2) {
    current_claw_close = PHY_CLAW_CLOSE_PRISM; target_base = RETRIEVE_BASE_PRISM; target_shoulder = RETRIEVE_SHOULDER_PRISM; target_elbow = RETRIEVE_ELBOW_PRISM;
    current_lift_shoulder = RETRIEVE_LIFT_SHOULDER_PRISM; current_lift_elbow = RETRIEVE_LIFT_ELBOW_PRISM;
  }
  
  tarShoulder = MAP_270(PHY_READY_SHOULDER); tarElbow = MAP_270(PHY_READY_ELBOW); tarClaw = MAP_270(PHY_CLAW_RELEASE); WaitUntilReach();
  tarBase = MAP_270(target_base); WaitUntilReach();
  tarShoulder = MAP_270(target_shoulder); tarElbow = MAP_270(target_elbow); WaitUntilReach(); delay(300); 
  tarClaw = MAP_270(current_claw_close); WaitUntilReach(); delay(300); 
  tarShoulder = MAP_270(current_lift_shoulder); tarElbow = MAP_270(current_lift_elbow); WaitUntilReach(); delay(300);
  tarElbow = MAP_270(RETRIEVE_CARRY_ELBOW); tarShoulder = MAP_270(RETRIEVE_CARRY_SHOULDER); WaitUntilReach();
  tarBase = MAP_270(RETURN_BASE); WaitUntilReach(); delay(300);
  tarElbow = MAP_270(RETURN_ELBOW); WaitUntilReach();
  tarShoulder = MAP_270(RETURN_SHOULDER); WaitUntilReach();
  tarClaw = MAP_270(PHY_CLAW_RELEASE); WaitUntilReach(); delay(300);
  tarShoulder = MAP_270(PHY_READY_SHOULDER); tarElbow = MAP_270(PHY_READY_ELBOW);tarClaw = MAP_270(PHY_READY_CLAW); WaitUntilReach();
  tarBase = MAP_270(PHY_READY_BASE);  WaitUntilReach();
  tarBase = MAP_270(FINISH_BASE); WaitUntilReach();
  tarShoulder = MAP_270(FINISH_SHOULDER); tarElbow = MAP_270(FINISH_ELBOW); tarClaw = MAP_270(FINISH_CLAW); WaitUntilReach();
}

// 舵机平滑移动
void UpdateServoSmooth() {
  static unsigned long lastMove = 0;
  if (millis() - lastMove >= SPEED_DELAY) {
    lastMove = millis();
    if (curBase < tarBase) curBase++; else if (curBase > tarBase) curBase--;
    if (curShoulder < tarShoulder) curShoulder++; else if (curShoulder > tarShoulder) curShoulder--;
    if (curElbow < tarElbow) curElbow++; else if (curElbow > tarElbow) curElbow--;
    if (curClaw < tarClaw) curClaw++; else if (curClaw > tarClaw) curClaw--;
    servoWriteDeg(baseS, curBase); servoWriteDeg(shoulderS, curShoulder);
    servoWriteDeg(elbowS, curElbow); servoWriteDeg(clawS, curClaw);
  }
}

void WaitUntilReach() {
  while (curBase!=tarBase || curShoulder!=tarShoulder || curElbow!=tarElbow || curClaw!=tarClaw) { UpdateServoSmooth(); }
}
