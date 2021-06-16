//#define SERIAL_DEBUG
#define BUZ_DEBUG
//#define SERVO_MODE

#ifdef SERVO_MODE
#include <Servo.h>
#endif

#define ENA 5         // 모터 제어 드라이버의 ENA, Shield의 S5 핀 연결
#define IN1 3         // 모터 제어 드라이버의 IN1, Shield의 S3 핀 연결
#define IN2 4         // 모터 제어 드라이버의 IN2, Shield의 S4 핀 연결
#define ENB 6         // 모터 제어 드라이버의 ENB, Shield의 S6 핀 연결
#define IN3 7         // 모터 제어 드라이버의 IN3, Shield의 S7 핀 연결
#define IN4 8         // 모터 제어 드라이버의 IN4, Shield의 S8 핀 연결
#define SL  A4        // 왼쪽 라인트레이서 센서 A4 핀 연결
#define SC  A2        // 중앙 라인트레이서 센서 A2 핀 연결
#define SR  A0        // 오른쪽 라인트레이서 센서 A0 핀 연결
#define BT  A1        // 시작 버튼
#define BUZ 2         // DEBUG 부저
#define TRIG1 10      // 초음파센서1 TRIG
#define ECHO1 9       // 초음파센서1 ECHO
#define TRIG2 13      // 초음파센서2 TRIG
#define ECHO2 12      // 초음파센서2 ECHO
#define THRESHOLD_SL  300   // 왼쪽 라인트레이서 센서 THRESHOLD 값
#define THRESHOLD_SC  300   // 중앙 라인트레이서 센서 THRESHOLD 값
#define THRESHOLD_SR  300   // 오른쪽 라인트레이서 센서 THRESHOLD 값
#define FREQ  880     // 부저음 주파수

#ifdef SERVO_MODE
#define SERVO 11      // 서보모터 Shield의 S11 핀 연결
#endif

typedef enum _dir {
    STOP = -1,
    FORWARD = 0,
    LEFT = 90,
    RIGHT = -90,
    BACK = 180
} edir;

typedef enum _kind {
    WL,
    WR,
    BL,
    BR,
    LINE
} ekind;

typedef enum _sensor {
    LL = 1,
    CC = 2,
    RR = 4
} esensor;

typedef enum _map {
    W,
    U,
    B,
    O,
    K
} emap;

typedef struct _vector {
    int x, y;
} vector;

#ifdef SERVO_MODE
Servo servo;
int angle = 0;
#endif

vector loc = {1, 1};        // 위치
vector vec = {0, 1};        // 시작 방향 위쪽
emap MAP[9][9] = {          // U: 모름, W: 흰색, B: 검은색, O: 벽, K: 지나감
    {W, W, W, W, W, W, W, W, W},
    {W, U, U, U, U, U, U, U, W},
    {W, U, W, U, W, U, W, U, W},
    {W, U, U, U, U, U, U, U, W},
    {W, U, W, U, W, U, W, U, W},
    {W, U, U, U, U, U, U, U, W},
    {W, U, W, U, W, U, W, U, W},
    {W, U, U, U, U, U, U, U, W},
    {W, W, W, W, W, W, W, W, W}
};

ekind go[2] = {BR, WR};
edir work = STOP;
esensor SIDE = LL;

edir dir(ekind);
void act(edir, int);

void motor(int, int, int);      // 일정 시간 동안 모터 회전
void line(ekind, int, int);     // 교차로 한 칸 진행
void left(int, int);            // 좌회전
void right(int, int);           // 우회전
void linedelay(ekind, int, int, unsigned long);     // 일정 시간 라인 타기
void toBlack(int, int, int);    // 검은색 인식 할 때까지 모터 회전
void toWhite(int, int, int);    // 흰색 인식 할 때까지 모터 회전
bool ultrasonic(edir, int);    // 초음파센서 감지여부
bool isSensorsOnBlack(int);     // 라인트레이서 센서 검은색 인식 여부
bool isSensorsOnWhite(int);     // 라인트레이서 센서 흰색 인식 여부
bool isObstacle();              // 전방에 벽 여부 확인
void setKind();
void recordMap(edir, int);      // 맵 기록
void printMap();                // 맵 출력
int getRelLocation(edir);       // 남은 교차점 확인
bool isVec(int, int);           // 벡터 방향 확인
vector rotationT(vector, int);  // 회전변환

bool isSL();                    // 왼쪽 라인트레이서 센서 검은색 인식 여부
bool isSC();                    // 중앙 라인트레이서 센서 검은색 인식 여부
bool isSR();                    // 오른쪽 라인트레이서 센서 검은색 인식 여부
void setSpd(int, int);          // 양쪽 모터 속도 설정
void motor_FL(int);             // 왼쪽 모터 정회전
void motor_FR(int);             // 오른쪽 모터 정회전
void motor_BL(int);             // 왼쪽 모터 역회전
void motor_BR(int);             // 오른쪽 모터 역회전

void setup() {
#ifdef SERIAL_DEBUG
    Serial.begin(9600);
#endif
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(TRIG1, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(TRIG2, OUTPUT);
    pinMode(ECHO2, INPUT);
    pinMode(BUZ, OUTPUT);
//    pinMode(BT, INPUT);
#ifdef SERVO_MODE
    servo.attach(SERVO);
    servo.write(angle);
#endif
}

void loop() {
//    while (!digitalRead(BT));
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
/*--------------code start--------------*/
    delay(500);
    int spd = 120, tspd = 150, tspd2 = 150, twspd = 60;
#ifdef SERIAL_DEBUG
    Serial.println("START");
#endif
    while (loc.x!=7 || loc.y!=7) {
        if (SIDE == LL) {
            if (work == RIGHT)
                recordMap(FORWARD, 1);
            else {
                recordMap(FORWARD, 4);
                recordMap(LEFT, 1);
            }
#ifdef SERIAL_DEBUG
            printMap();
#endif
            if (MAP[loc.x][loc.y] == U &&
                !isObstacle() &&
                (isVec(0, 1) || isVec(-1, 0))) {
                int len = getRelLocation(FORWARD);
                for (int i=0; i<len; i++) {
                    setKind();
                    line(go[0], spd, tspd);
                    if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                    else line(go[1], spd, tspd);
                }
                MAP[loc.x][loc.y] = K;
                right(tspd, tspd2);
                work = RIGHT;
#ifdef SERIAL_DEBUG
                Serial.println("No Obstacle");
#endif
            } else {
                if (MAP[loc.x-vec.y][loc.y+vec.x] == B) {
                    left(tspd, tspd2);
                    setKind();
                    line(go[0], spd, tspd);
                    if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                    else line(go[1], spd, tspd);
                    work = FORWARD;
#ifdef SERIAL_DEBUG
                    Serial.println("Left and go");
#endif
                } else if (MAP[loc.x-vec.y][loc.y+vec.x] == O ||
                           MAP[loc.x-vec.y][loc.y+vec.x] == W) {
                    if (MAP[loc.x+vec.x][loc.y+vec.y] == B) {
                        setKind();
                        line(go[0], spd, tspd);
                        if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                        else line(go[1], spd, tspd);
                        work = FORWARD;
#ifdef SERIAL_DEBUG
                        Serial.println("go");
#endif
                    } else if (MAP[loc.x+vec.x][loc.y+vec.y] == O ||
                               MAP[loc.x+vec.x][loc.y+vec.y] == W) {
                        right(tspd, tspd2);
                        work = RIGHT;
#ifdef SERIAL_DEBUG
                        Serial.println("Right");
#endif
                    }
                }
            }
            if (loc.x == 7) {
#ifdef SERVO_MODE
                SIDE = RR;
                angle = 180;
                servo.write(angle);
                delay(100);
#endif
            }
        }
        if (SIDE == RR) {
            if (work == LEFT)
                recordMap(FORWARD, 1);
            else {
                recordMap(FORWARD, 4);
                recordMap(RIGHT, 1);
            }
#ifdef SERIAL_DEBUG
            printMap();
#endif
            if (MAP[loc.x][loc.y] == U &&
                !isObstacle() &&
                (isVec(0, -1) || isVec(1, 0))) {
                int len = getRelLocation(FORWARD);
                for (int i=0; i<len; i++) {
                    setKind();
                    line(go[0], spd, tspd);
                    if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                    else line(go[1], spd, tspd);
                }
                MAP[loc.x][loc.y] = K;
                left(tspd, tspd2);
                work = LEFT;
#ifdef SERIAL_DEBUG
                Serial.println("No Obstacle");
#endif
            } else {
                if (MAP[loc.x+vec.y][loc.y-vec.x] == B) {
                    right(tspd, tspd2);
                    setKind();
                    line(go[0], spd, tspd);
                    if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                    else line(go[1], spd, tspd);
                    work = FORWARD;
#ifdef SERIAL_DEBUG
                    Serial.println("Right and go");
#endif
                } else if (MAP[loc.x+vec.y][loc.y-vec.x] == O ||
                           MAP[loc.x+vec.y][loc.y-vec.x] == W) {
                    if (MAP[loc.x+vec.x][loc.y+vec.y] == B) {
                        setKind();
                        line(go[0], spd, tspd);
                        if (go[1] == LINE) toWhite(twspd, twspd, LL+RR);
                        else line(go[1], spd, tspd);
                        work = FORWARD;
#ifdef SERIAL_DEBUG
                        Serial.println("go");
#endif
                    } else if (MAP[loc.x+vec.x][loc.y+vec.y] == O ||
                               MAP[loc.x+vec.x][loc.y+vec.y] == W) {
                        left(tspd, tspd2);
                        work = LEFT;
#ifdef SERIAL_DEBUG
                        Serial.println("Left");
#endif
                    }
                }
            }
            if (loc.y == 7) {
#ifdef SERVO_MODE
                SIDE = LL;
                angle = 0;
                servo.write(angle);
                delay(100);
#endif
            }
        }
    }
#ifdef SERIAL_DEBUG
    Serial.println("FINISH");
#endif
    delay(10000);
    
/*---------------code end---------------*/
}

void printMap() {
    for (int i=0; i<12; i++)
        Serial.print("=");
    Serial.println("");
    for (int i=0; i<9; i++) {
        for (int j=0; j<9; j++) {
            Serial.print(MAP[j][8-i]);
            Serial.print(" ");
        }
        Serial.println("");
    }
    Serial.println("");
}

void recordMap(edir dir, int n) {
    int i;
    int distance[3] = {40, 100, 150};
    int len = getRelLocation(dir);
    vector v = rotationT(vec, dir);
    
    for (i=0; i<len && i<n; i++) {
        if (ultrasonic(dir, distance[i])) {
            for (int j=0; j<i; j++)
                MAP[loc.x+v.x*(2*j+1)][loc.y+v.y*(2*j+1)] = B;
            MAP[loc.x+v.x*(2*i+1)][loc.y+v.y*(2*i+1)] = O;
            break;
        }
    }
    if (i == len)
        for (int j=0; j<i; j++)
            MAP[loc.x+v.x*(2*j+1)][loc.y+v.y*(2*j+1)] = B;
    if (i == n)
        for (int j=0; j<i; j++)
            MAP[loc.x+v.x*(2*j+1)][loc.y+v.y*(2*j+1)] = B;
}

bool isObstacle() {
    int len = getRelLocation(FORWARD);
    for (int i=0; i<len; i++)
        if (MAP[loc.x+vec.x*(2*i+1)][loc.y+vec.y*(2*i+1)] != B)
            return true;
    return false;
}

int getRelLocation(edir dir) {
    vector v = rotationT(vec, dir);
    int len = (v.x*loc.x+v.y*loc.y+1)/2;
    return len>0? 4-len:-len;
}

void setKind() {
    if (getRelLocation(LEFT) == 0) {
        go[0] = BR;
        if (getRelLocation(FORWARD) == 1) go[1] = LINE;
        else go[1] = WR;
    } else if (getRelLocation(RIGHT) == 0) {
        go[0] = BL;
        if (getRelLocation(FORWARD) == 1) go[1] = LINE;
        else go[1] = WL;
    } else {
        go[0] = BR;
        if (getRelLocation(FORWARD) == 1) go[1] = LINE;
        else go[1] = WR;
    }
}

void line(ekind kind, int spd, int tspd) {
    int d;
    if (kind == BL || kind == BR)
        MAP[loc.x][loc.y] = K;
    if (kind == BL) toWhite(-tspd, tspd, LL);
    if (kind == BR) toWhite(tspd, -tspd, RR);
    do {
        d = dir(kind);
        act(d, spd, tspd);
    } while (d != STOP);
    if (kind == BL || kind == BR) {
        loc.x += 2*vec.x;
        loc.y += 2*vec.y;
    }
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

void linedelay(int spd, int tspd, unsigned long t) {
    unsigned long T = millis();
    int d;
    do {
        d = dir(LINE);
        act(d, spd, tspd);
    } while (millis() - T < t);
    setSpd(0, 0);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

void left(int spd, int spd2) {
    toWhite(-spd, spd, LL);
    toBlack(-spd, spd, LL);
    toWhite(-spd2, spd2, LL);
    if (getRelLocation(LEFT) == 0) {
        if (getRelLocation(FORWARD) == 3) vec = rotationT(vec, 270);
        else vec = rotationT(vec, 180);
    }
    else vec = rotationT(vec, 90);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

void right(int spd, int spd2) {
    toWhite(spd, -spd, RR);
    toBlack(spd, -spd, RR);
    toWhite(spd2, -spd2, RR);
    if (getRelLocation(RIGHT) == 0) {
        if (getRelLocation(FORWARD) == 3) vec = rotationT(vec, -270);
        else vec = rotationT(vec, -180);
    }
    else vec = rotationT(vec, -90);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

bool isVec(int x, int y) {
    if (vec.x==x && vec.y==y) return true;
    else return false;
}

vector rotationT(vector v, int angle) {
    int tmp = v.x;
    switch (angle) {
        case -90:
        case 270:
            v.x = v.y; v.y = -tmp;
            break;
        case 90:
        case -270:
            v.x = -v.y; v.y = tmp;
            break;
        case 180:
        case -180:
            v.x *= -1; v.y *= -1;
            break;
    }
    return (vector){v.x, v.y};
}

edir dir(ekind kind) {
    bool ll = isSL();
    bool cc = isSC();
    bool rr = isSR();
    if (kind == WL) {
        if (!ll) return STOP;
        if (cc && rr) return FORWARD;
        if (cc) return LEFT;
        if (rr) return RIGHT;
        return STOP;
    }
    if (kind == WR) {
        if (!rr) return STOP;
        if (ll && cc) return FORWARD;
        if (cc) return RIGHT;
        if (ll) return LEFT;
        return STOP;
    }
    if (kind == BL) {
        if (ll) return STOP;
        if (cc && rr) return FORWARD;
        if (cc) return LEFT;
        if (rr) return RIGHT;
        return STOP;
    }
    if (kind == BR) {
        if (rr) return STOP;
        if (ll && cc) return FORWARD;
        if (cc) return RIGHT;
        if (ll) return LEFT;
        return STOP;
    }
    if (kind == LINE) {
        if (ll && cc &&rr) return STOP;
        if (!ll && !cc && !rr) return STOP;
        if (ll) return LEFT;
        if (rr) return RIGHT;
    }
}

void act(edir d, int spd, int tspd) {
    if (d == STOP) setSpd(0, 0);
    if (d == FORWARD) setSpd(spd, spd);
    if (d == LEFT) setSpd(0, tspd);
    if (d == RIGHT) setSpd(tspd, 0);
    if (d == BACK) setSpd(-spd, -spd);
}

void toBlack(int spl, int spr, int sensors) {
    setSpd(spl, spr);
    while (!isSensorsOnBlack(sensors));
    setSpd(0, 0);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

void toWhite(int spl, int spr, int sensors) {
    setSpd(spl, spr);
    while (!isSensorsOnWhite(sensors));
    setSpd(0, 0);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

bool ultrasonic(edir d, int range) {
    long cm;
    if (d == LEFT) {
        digitalWrite(TRIG1, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG1, LOW);
        cm = pulseIn(ECHO1, HIGH) / 58;
    }
    if (d == FORWARD) {
        digitalWrite(TRIG2, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG2, LOW);
        cm = pulseIn(ECHO2, HIGH) / 58;
    }
#ifdef SERIAL_DEBUG
    Serial.print("distance: ");
    Serial.println(cm);
#endif
    if (cm < range) return true;
    else return false;
}

bool isSL() {
    int sens = (analogRead(SL)+analogRead(SL)+analogRead(SL))/3;
#ifdef SERIAL_DEBUG
    Serial.print("left: ");
    Serial.println(sens);
#endif
    return sens>THRESHOLD_SL ? true:false;
}

bool isSC() {
    int sens = (analogRead(SC)+analogRead(SC)+analogRead(SC))/3;
#ifdef SERIAL_DEBUG
    Serial.print("center: ");
    Serial.println(sens);
#endif
    return sens>THRESHOLD_SC ? true:false;
}

bool isSR() {
    int sens = (analogRead(SR)+analogRead(SR)+analogRead(SR))/3;
#ifdef SERIAL_DEBUG
    Serial.print("right: ");
    Serial.println(sens);
#endif
    return sens>THRESHOLD_SR ? true:false;
}

bool isSensorsOnBlack(int s) {
    bool res = true;
    if (s & LL) res &= isSL();
    if (s & CC) res &= isSC();
    if (s & RR) res &= isSR();
    return res;
}

bool isSensorsOnWhite(int s) {
    bool res = false;
    if (s & LL) res |= isSL();
    if (s & CC) res |= isSC();
    if (s & RR) res |= isSR();
    return !res;
}

void setSpd(int spl, int spr) {
    if (spl > 255) spl = 255;
    else if (spl < -255) spl = -255;
    if (spr > 255) spr = 255;
    else if (spr < -255) spr = -255;
    
    if (spl > 0) motor_FL(spl);
    else motor_BL(-spl);
    if (spr > 0) motor_FR(spr);
    else motor_BR(-spr);
}

void motor(int spl, int spr, int t) {
    setSpd(spl, spr);
    delay(t);
    setSpd(0, 0);
#ifdef BUZ_DEBUG
    tone(BUZ, FREQ, 50);
#endif
}

void motor_FL(int spd) {
#ifdef SERIAL_DEBUG
    Serial.println("Right Motor Forward");
#endif
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, spd);
}

void motor_FR(int spd) {
#ifdef SERIAL_DEBUG
    Serial.println("Left Motor Forward");
#endif
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, spd);
}

void motor_BL(int spd) {
#ifdef SERIAL_DEBUG
    Serial.println("Right Motor Back");
#endif
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, spd);
}

void motor_BR(int spd) {
#ifdef SERIAL_DEBUG
    Serial.println("Left Motor Back");
#endif
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, spd);
}