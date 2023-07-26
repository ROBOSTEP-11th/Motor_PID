#include "mbed.h"
#include "mext_encoder_stm32/Encoder.h"
#define RESOLUTION 2048
#include "mbed.h"
#include "CalPID.h"
#include "NsPwmOut.h"
#include "MotorController.h"

using namespace mext;

// 制御周期[s]
#define DELTA_T 0.05
// dutyの上限値
#define DUTY_MAX 0.7
// 角度の上限 速度制御の場合は宣言のみ
#define OMEGA_MAX 6
// 角速度を格納する配列の要素数
#define NUM_DATA 500

// P→D→Iの順に調整
// 速度制御のPID (kp, ki, kd, 制御周期, dutyの上限値)
CalPID speed_pid(0.0, 0.0000, 0.0000, DELTA_T, DUTY_MAX);
// 角度制御のPID (kp, ki, kd, 制御周期, 角度の上限値) 
// 速度制御の場合は宣言のみ
CalPID angle_omega_pid(0.0, 0.0, 0.0, DELTA_T, OMEGA_MAX);

//エンコーダ（A層, B層, 分解能）
//モーター（正転, 逆転, PWM周期, エンコーダ, 速度制御のPID, 角度制御のPID)
Ec ec(PA_9, PA_8, RESOLUTION);
MotorController motor(PA_3, PA_1, DELTA_T, ec,speed_pid, angle_omega_pid);

Ticker ticker;  //割り込みタイマー

float target_speed = 0;

// 角速度を保存する変数と関数
float omega_saved[NUM_DATA] = {};
int omega_count = 0;

void saveOmega() {
    if (omega_count < NUM_DATA) {
        omega_saved[omega_count] = ec.getOmega();
        omega_count++;
    }
}

// データ保存の周期調整用
// データ保存の周期 = 制御周期 * SAVE_COUNT_THRESHOLD
int save_count = 0;
#define SAVE_COUNT_THRESHOLD 0

//保存したデータをprintf
void displayData() {
    for(int i = 0; i < omega_count; i++) {
        // 角速度を記録
        printf("%f,", omega_saved[i]);
        // printfは重い処理 マイコンが落ちないように
        thread_sleep_for(50);//printf重いのでマイコンが落ちないように
    }
    omega_count=0;
}

// 速度制御とデータ保存
void speedControll() {
    motor.Sc(target_speed);
    save_count++;
    if(save_count >= SAVE_COUNT_THRESHOLD) {
        saveOmega();
        save_count = 0;
    }
}

int main ()
{
    //ギア比 タイヤ:エンコーダ＝１:rとしたときのrの値
    ec.setGearRatio((double) 7/4);
    
    // 角速度とduty比の関係式設定
    // 横軸をタイヤの角速度[rad/s]とduty比とした時の傾きとy切片
    // 正転時の傾き, y切片, 逆転時の傾き, y切片
    motor.setEquation(0.043168,0.024875,-0.04398,0.016883);

    // 事前にプログラムをマイコンに読み込んでおく
    // 緊急停止スイッチで切った状態で電源をつなぎ、マイコンをリセットする
    // 改行をしたら緊急停止スイッチを解除する
    printf("\r\n");
    thread_sleep_for(3000);
    //STARTが書かれた時点で開始する
    printf("START\r\n");

    // 目標角速度を設定
    target_speed = 4;
    // 割り込みタイマー
    ticker.attach(&speedControll,50ms);

    // モーターを回す時間
    thread_sleep_for(3000);
    // 割り込みを止める
    ticker.detach();
    // モーターを止める
    motor.stop();
    // 角速度のデータを表示
    // エクセルなどでグラフ化し調整
    displayData();
    printf("\r\n");
}
