#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <stdint.h>
#include <sys/times.h>
#include <time.h>
#include <linux/kernel.h>
#include <sys/sysinfo.h>
#include <unistd.h>
#include <sys/utsname.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <pigpiod_if2.h>
#include "MPU9250Operator.h"
#include "MPU9250.h"

#include "wiimote.hpp"
#include "OmniOperator.hpp"
#include <cwiid.h>

#define MSEC 1000000
#define INTERVAL_200MSEC 200*MSEC
#define INTERVAL_10MSEC 10*MSEC
#define PI 3.14159265359
#define MAXPULSE 100

#define abs_(x) ((x)<0.0 ? (-1*(x)) : (x) )

wiimote_c wiimote1; //クラスにvolatileは要らないらしい．
wiimote_c wiimote2; //クラスにvolatileは要らないらしい．
OmniOperator *omniOperator1 = NULL;
MPU9250Operator *mMpu9250Operator = NULL;

using namespace std;

const uint8_t TOP_PIN_ID = 4;
const uint8_t LEFT_PIN_ID = 27;
const uint8_t RIGHT_PIN_ID = 22;

const uint8_t LED1_PIN_ID = 6;
const uint8_t LED2_PIN_ID = 5;

const int SIGNAL_10MS = 10;
const int SIGNAL_200MS = 200;
const float MIN_WEIGHT = 5; //バランスボードに人が乗っていないと判定する閾値

enum MODE {
    NUNCHUK,
    BALANCEBOARD,
};

MODE actMode = BALANCEBOARD;
volatile bool quit_flag=false;

struct BF {
    unsigned int mode : 2;
    unsigned int p0 : 1; //CSをActiveLowにするかどうか
    unsigned int p1 : 1; //
    unsigned int p2 : 1; //
    unsigned int u0 : 1; //CSをユーザ操作するかどうか
    unsigned int u1 : 1; //
    unsigned int u2 : 1; //
    unsigned int A : 1; //mainかauxiliaryか
    unsigned int W : 1; //3wireか4wireか．mainのみ
    unsigned int nnnn : 4; //何byte読んだら次の読み込みに進むか
    unsigned int T : 1; //MOSIのMSB/LSBファースト設定
    unsigned int R : 1; //MISOのMSB/LSBファースト設定
    unsigned int bbbbbb : 6; //1文字あたりのbit数
};

typedef union {
    struct BF field;
    unsigned int asInt;
} SPI_FLAGS;

void interruptedFunc(int sig, siginfo_t *si, void *uc);
void quitProgram(int piId);

void err(cwiid_wiimote_t *wiimote, const char *s, va_list ap)
{
	if (wiimote) printf("%d:", cwiid_get_id(wiimote));
	else printf("-1:");
	vprintf(s, ap);
	printf("\n");
}

void timer_init(int interval, int sigId) {
    //シグナルに番号を付与できるモダンな？やりかたに変更
    //http://umezawa.dyndns.info/wordpress/?p=3174
    struct itimerspec timerSpec;
    struct sigevent se;
    timerSpec.it_value.tv_nsec = interval;
    timerSpec.it_value.tv_sec = 0;
    timerSpec.it_interval.tv_nsec = interval;
    timerSpec.it_interval.tv_sec = 0;

    timer_t timerid_signal;
    memset(&se, 0, sizeof (se));
    se.sigev_value.sival_int = sigId;
    se.sigev_signo = SIGALRM;
    se.sigev_notify = SIGEV_SIGNAL;
    timer_create(CLOCK_REALTIME, &se, &timerid_signal);
    timer_settime(timerid_signal, 0, &timerSpec, NULL);
}

void interupt_init() {
    struct sigaction action;
    memset(&action, 0, sizeof (action)); /* actionの内容を0でクリア */

    action.sa_sigaction = interruptedFunc; //呼び出す関数のセット．こっちの方式だと呼び出し元で設定した番号を引ける
    action.sa_flags = SA_SIGINFO; //sa_handlerではなくsa_sigactinを指定するための設定．
    action.sa_flags |= SA_RESTART; /* システムコールが中止しない */
    sigemptyset(&action.sa_mask); //maskの中身を0にクリア．maskに登録しておくと割り込みが重なった時に保留してくれる．
    if (sigaction(SIGALRM, &action, NULL) < 0)//第一引数でシグナルの種類を指定．今回はアラームの時間満了で呼ばれる．第三引数にsigaction型をセットしておくと，ひとつ古い値が返ってくる．今回は破棄
    {
        perror("sigaction error");
        exit(1);
    }
}

int calcByNunchuk(wiimote_c *wiimote, float xyVals[2]) {
    if (wiimote->state.ext_type != CWIID_EXT_NUNCHUK) {
        return -1;
    }

    float x_val = float(wiimote->state.ext.nunchuk.stick[CWIID_X]) / 127 - 1; //-1~1の値が得られる
    float y_val = float(wiimote->state.ext.nunchuk.stick[CWIID_Y]) / 127 - 1;

    //printf("%f\t",x_val);
    /*ニュートラルのずれを無視*/
    if (abs_(x_val) < 5.0 / 127) {
        x_val = 0;
    }
    if (abs_(y_val) < 5.0 / 127) {
        y_val = 0;
    }
    xyVals[0] = x_val;
    xyVals[1] = y_val;
    return 0;
}

int calcByBalanceBoard(wiimote_c *wiimote, float xyVals[2]) {
    if (wiimote->state.ext_type != CWIID_EXT_BALANCE) {
        return -1;
    }

    wiimote->updateWeightFromBlanceBoard();
    if (wiimote->getTotalWeightFromBalanceBoard() < MIN_WEIGHT) {
        return 0;
    }

    wiimote->getRevisedPositionFromBalanceBoard(xyVals);
    printf("balanceX:%f\r\n", xyVals[0]);
    printf("balanceY:%f\r\n", xyVals[1]);

    //printf("%f\t",x_val);
    /*ニュートラルのずれを無視*/
    if (abs_(xyVals[0]) < 0.2) {
        xyVals[0] = 0.;
    }
    if (abs_(xyVals[1]) < 0.2) {
        xyVals[1] = 0.;
    }
    if (xyVals[1] < 0) {
        xyVals[1] *= 1.2;
    }

    return 0;
}

float addReviseRotation(wiimote_c *wiimote, float rate = 0.3) {
    if (actMode == NUNCHUK) {
        return 0;
    }

    if (wiimote->state.ext_type != CWIID_EXT_NUNCHUK) {
        return 0;
    }

	float x_val=float(wiimote->state.ext.nunchuk.stick[CWIID_X])/127-1;//-1~1の値が得られる
//    float y_val = float(wiimote->state.ext.nunchuk.stick[CWIID_Y]) / 127 - 1;
    if (abs_(x_val) < 5.0 / 127) {
        x_val = 0;
    }
//    printf("stickX:%f\r\n", x_val);
    return -x_val*rate;
}

void interruptedFunc(int sig, siginfo_t *si, void *uc) {
    static uint8_t buttonA = 0;
    static uint8_t buttonC = 0;
    switch (si->si_value.sival_int) {
        case SIGNAL_10MS:
            break;
        case SIGNAL_200MS:
#ifdef GYRO_TEST
            int16_t accelDatas[3] = {0};
            int16_t gyroDatas[3] = {0};
            //	    mMpu9250Operator->readAccelData(accelDatas);
            mMpu9250Operator->readGyroData(gyroDatas);

            //    printf("accelX:%hd\r\n",accelDatas[0]);
            //    printf("accelY:%hd\r\n",accelDatas[1]);
            //	    printf("accelZ:%hd\r\n",accelDatas[2]);
            printf("gyroX:%hd\r\n", gyroDatas[0]);
            //	    printf("gyroY:%f\r\n",(float)gyroDatas[1])*mMpu9250Operator->gRes;
            //	    printf("gyroZ:%f\r\n",(float)gyroDatas[2])*mMpu9250Operator->gRes;
            return;
#endif
            /*リモコンの値を取得*/
			int err=0;
			err = wiimote1.get_state();
            err = wiimote2.get_state();

//	if(wiimote1.state.buttons & CWIID_BTN_HOME){
//		quit_flag = true;
//    }

			if(quit_flag == true){
				return;
			}
            float xyVals[2] = {0., 0.};
            switch (actMode) {
                case NUNCHUK:
                    calcByNunchuk(&wiimote1, xyVals);
                    break;
                case BALANCEBOARD:
					//センターポイント初期化処理．こっちで調整するより，立ち位置見直してもらったほうが事故が少ない．
					/*
                    //ボタン押下処理．暫定対策．
                    if ((buttonA^(wiimote1.state.buttons & CWIID_BTN_A)) && (wiimote1.state.buttons & CWIID_BTN_A)) {
                        wiimote2.initCriteriaOfBalanceBoard();
                        puts("INIT CALLED");
                    } else if ((buttonC^(wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C)) && (wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C)) {
                        wiimote2.initCriteriaOfBalanceBoard();
                        puts("INIT CALLED");
                    }
                    buttonA = wiimote1.state.buttons & CWIID_BTN_A;
                    buttonC = wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_C;
					 */

                    calcByBalanceBoard(&wiimote2, xyVals);
                    if (!(wiimote1.state.ext.nunchuk.buttons & CWIID_NUNCHUK_BTN_Z) && !(wiimote1.state.buttons & CWIID_BTN_B) ) {//ボタンを押している間だけ動く
                        xyVals[0] = xyVals[1] = 0;
                    }
                    break;
            }
            float rotation = addReviseRotation(&wiimote1);

            /*モーター回転量計算*/
            omniOperator1->move(xyVals[0], xyVals[1], rotation);
            break;
    }
}

//エラー発生時にコール．エラー点滅ののちプログラム終了
void quitProgram(int piId) {
    set_PWM_frequency(piId, LED1_PIN_ID, 5); //5Hz
    set_PWM_frequency(piId, LED2_PIN_ID, 5); //5Hz
    set_PWM_dutycycle(piId, LED1_PIN_ID, 127); //点滅開始
    set_PWM_dutycycle(piId, LED2_PIN_ID, 127); //点滅開始

    time_sleep(3);
    gpio_write(piId,LED1_PIN_ID,0);
    gpio_write(piId,LED2_PIN_ID,0);
    pigpio_stop(piId);
    exit(0);
}

int main(void) {
    int piId = pigpio_start(NULL, NULL); //ネットワーク越しに使えるっぽい．NULL指定時はローカルホストの8888ポート

    //起動確認LED，シャットダウンスイッチのセットアップ
    set_mode(piId, LED1_PIN_ID, PI_OUTPUT);
    set_pull_up_down(piId, LED1_PIN_ID, PI_PUD_OFF);
    set_PWM_frequency(piId, LED1_PIN_ID, 1); //1Hz//pigpioの仕様上，下限は5Hzらしい．
    set_PWM_range(piId, LED1_PIN_ID, 255); //点灯/点滅/消灯だけできればよいので適当.255はデフォルト値なので書かなくてもいいはず
//    set_PWM_dutycycle(piId, LED1_PIN_ID, 0);
    gpio_write(piId,LED1_PIN_ID,0);
    set_mode(piId, LED2_PIN_ID, PI_OUTPUT);
    set_pull_up_down(piId, LED2_PIN_ID, PI_PUD_OFF);
    set_PWM_frequency(piId, LED2_PIN_ID, 1); //1Hz
    set_PWM_range(piId, LED2_PIN_ID, 255); //点灯/点滅/消灯だけできればよいので適当.255はデフォルト値なので書かなくてもいいはず
//    set_PWM_dutycycle(piId, LED2_PIN_ID, 0);
    gpio_write(piId,LED2_PIN_ID,0);

    //オムニ操作部分のセットアップ
    omniOperator1 = new OmniOperator(piId, TOP_PIN_ID, LEFT_PIN_ID, RIGHT_PIN_ID);
    omniOperator1->init(1000, 1000); //1kHz,分解能1000
    omniOperator1->set_limit(60); //最大出力を90%に制限

#ifdef GYRO_TEST
    SPI_FLAGS spi_flags = {0};
    spi_flags.field.mode = 0x03; //mode3
    int spi_handle = spi_open(piId, 0, 100000, spi_flags.asInt);

    mMpu9250Operator = new MPU9250Operator(piId, spi_handle);
    float gyroBias[3] = {0.};
    float accelBias[3] = {0.};
    mMpu9250Operator->calibrateMPU9250(gyroBias, accelBias);
    for (int i = 0; i < 3; i++) {
        printf("gyroBias[%d]=%f\r\n", i, gyroBias[i]);
    }
    for (int i = 0; i < 3; i++) {
        printf("accelBias[%d]=%f\r\n", i, accelBias[i]);
    }
    mMpu9250Operator->initMPU9250();
#endif

    //    /* Connect to the wiimote */
    printf("Put Wiimote in discoverable mode now (press 1+2)...\n");
    set_PWM_dutycycle(piId, LED1_PIN_ID, 127); //点滅開始
    if (wiimote1.open() == -1 ) {
        quitProgram(piId);
    }
    wiimote1.set_rpt_mode(CWIID_RPT_ACC);
    wiimote1.set_rpt_mode(CWIID_RPT_BTN);
    wiimote1.set_rpt_mode(CWIID_RPT_EXT);
    time_sleep(0.1);//設定反映までに少し時間がかかるらしい
    wiimote1.get_state();
    if(wiimote1.state.ext_type != CWIID_EXT_NUNCHUK){
	printf("connected to wrong device\r\n");
        quitProgram(piId);
    }
	wiimote1.set_err(err);

    set_PWM_dutycycle(piId, LED1_PIN_ID, 255); //点灯開始

    printf("again balance board (press 1+2)...\n");
    set_PWM_dutycycle(piId, LED2_PIN_ID, 127); //点滅開始
    if (wiimote2.open() == -1) {
        quitProgram(piId);
    }
    wiimote2.set_rpt_mode(CWIID_RPT_EXT);
    time_sleep(0.1);//設定反映までに少し時間がかかるらしい
    wiimote2.get_state();
    if(wiimote2.state.ext_type != CWIID_EXT_BALANCE){
	printf("connected to wrong device\r\n");
        quitProgram(piId);
    }
    set_PWM_dutycycle(piId, LED2_PIN_ID, 255); //点灯開始
    //cwiid_set_err(err);

    interupt_init();
    timer_init(INTERVAL_200MSEC, SIGNAL_200MS);

    puts("start");
    while (!quit_flag) {
    }
    puts("finish");

    gpio_write(piId,LED1_PIN_ID,0);
    gpio_write(piId,LED2_PIN_ID,0);
    pigpio_stop(piId);
    delete(omniOperator1);
    wiimote1.close();
    wiimote2.close();
#ifdef GYRO_TEST
    delete(mMpu9250Operator);
    spi_close(piId, spi_handle);
#endif

    return 0;
}
