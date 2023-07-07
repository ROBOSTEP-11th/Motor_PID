#include "mbed.h"
#include "mext_encoder_stm32/Encoder.h"
#define RESOLUTION 2048
#include "mbed.h"
#include "CalPID.h"
#include "NsPwmOut.h"
#include "MotorController.h"

using namespace mext;

//0.05秒ごとに角速度を取る
#define DELTA_T 0.05
//dutyの絶対値の上限を決めて暴走を防ぐ
//タイヤCのみ変えているのは多分ギア比が異なるためP制御がうまくかからなかったため
#define DUTY_MAX 0.5
#define OMEGA_MAX 6
//角速度を格納する配列の要素数
#define NUM_DATA 500

//速度制御のPID 前3つが係数なのでこれを調整 P→D→Iの順
CalPID speed_pid(0.0,0.0000,0.0000,DELTA_T,DUTY_MAX);
//角度制御のPIDなので不要ですがMotorContorollerの引数なので消さずに放置してください
CalPID angle_omega_pid(0.0,0.0,0.0,DELTA_T,OMEGA_MAX);
//エンコーダ（A層,B層,分解能）
//モーター（正転,逆転,PWM周期,エンコーダ,CALPIDの角速度PID,角度PID)
//タイヤA
Ec ec(PA_9, PA_8, RESOLUTION);
MotorController motor(PA_3, PA_1,DELTA_T,ec,speed_pid,angle_omega_pid);

Ticker ticker;  //割り込みタイマー

//角速度を保存する変数と関数
float omega_saved[NUM_DATA]= {};
int omega_count=0;
void saveOmega()
{
    if(omega_count<NUM_DATA) {
        omega_saved[omega_count]=ec.getOmega();
        omega_count++;
    }
}

float target_speed=0;
int save_count;//データ保存の周期調整用1kHzで保存すると1秒で1000個もデータが溜まってしまう
#define SAVE_COUNT_THRESHOLD 0
//モーターを目標角速度で動かそうとし、一定の間隔で角速度を配列に保存
void speedControll()
{
    motor.Sc(target_speed);
    save_count++;
    if(save_count>SAVE_COUNT_THRESHOLD) {
        saveOmega();
        save_count=0;
    }
}
void displayData()//保存したデータをprintf
{
    for(int i=0; i<omega_count; i++) {
        printf("%f,",omega_saved[i]);//角速度を記録
        ThisThread::sleep_for(50ms);//printf重いのでマイコンが落ちないように
    }
    omega_count=0;
}

int main ()
{
    //多分モーターの加速度の上限 大きくしておけばいいと言われたので10000000にした
    //motor.setAccelMax(10000000);
    //ギア比 タイヤ:エンコーダ＝１:rとしたときのrの値
    ec.setGearRatio((double) 7/4);
    
    //角速度とduty比の関係式設定
    //横軸をタイヤの角速度[rad/s]とduty比とした時の傾きとy切片
    //正転時の傾き,y切片,逆転時の傾き,y切片
    motor.setEquation(0.043168,0.024875,-0.04398,0.016883);

    //事前にプログラムをマイコンに読み込んでおく
    //緊急停止スイッチで切った状態で電源をつなぎ、マイコンをリセットする
    //改行をしたら緊急停止スイッチを解除する
    printf("\r\n");
    ThisThread::sleep_for(3s);
    //STARTが書かれた時点で開始する
    printf("START\r\n");

    target_speed=4;//ここで目標角速度を設定
    ticker.attach(&speedControll,50ms);//割り込みタイマーをオンにする

    ThisThread::sleep_for(3s);//モーターが割り込みでまわり10秒分のデータを取る
    ticker.detach();//割り込みを止める（止めないとモーターを動かし続けてしまう）
    motor.stop();//モーターを止める(dutyを0にしている)
    displayData();//角速度のデータが表示されるのでエクセルにいれてグラフにする
    printf("\r\n");

}
