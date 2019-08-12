#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <bluetooth/bluetooth.h>
#include <cwiid.h>
#include <pigpiod_if2.h>
#include "wiimote.hpp"


wiimote_c::wiimote_c() : bdaddr(), state(), rpt_mode(0) {
}//コンストラクタ,コロン以降は初期化子

void wiimote_c::set_err(cwiid_err_t err) {
	cwiid_set_err(err);
}

int wiimote_c::open() {
    if (!(body = cwiid_open(&bdaddr, 0))) {
        fprintf(stderr, "Unable to connect to wiimote\n");
		return -1;
    }
    return 0;
}

int wiimote_c::open(int timeout) {
    if (!(body = cwiid_open_timeout(&bdaddr, 0,timeout))) {
        fprintf(stderr, "Unable to connect to wiimote\n");
		return -1;
    }
    return 0;
}

int wiimote_c::close() {
	return cwiid_close(body);
}

int wiimote_c::toggle_led(unsigned char led_no) {
    //puts("toggle_LED called\n");
    if (led_no >= 4)
        return -1;

    uint8_t led_state = state.led;
    toggle_bit(led_state, (1 << led_no));
	int ret=cwiid_set_led(body, led_state);
    if (ret){
        fprintf(stderr, "Error setting LEDs \n");
		return ret;
	}
    get_state();
    return 0;
}

int wiimote_c::set_rpt_mode(unsigned char lcl_rpt_mode) {
    rpt_mode |= lcl_rpt_mode;
	int ret = cwiid_set_rpt_mode(body, rpt_mode);
    if (ret) {
        fprintf(stderr, "Error setting report mode\n");
		return ret;
    }
    return get_state();
}

int wiimote_c::clr_rpt_mode(unsigned char lcl_rpt_mode) {
    rpt_mode &= ~lcl_rpt_mode;
	int ret = cwiid_set_rpt_mode(body, rpt_mode);
    if (ret) {
        fprintf(stderr, "Error setting report mode\n");
		return ret;
    }
    return get_state();
}

int wiimote_c::get_state() {
	return cwiid_get_state(body, &state) ;
}

void wiimote_c::set_enable(int flags) {
    cwiid_enable(body, flags);
    get_state();
}

void wiimote_c::set_disable(int flags) {
    cwiid_disable(body, flags);
    get_state();
}

void wiimote_c::get_motionplus_state() {

    //set_enable(CWIID_FLAG_MOTIONPLUS);
    if (state.ext_type == CWIID_EXT_MOTIONPLUS) {
        for (int i = 0; i < 3; i++) {
            angle_rates[i] = state.ext.motionplus.angle_rate[i];
            printf("angle_rates[%d]=[%hd]\n", i, angle_rates[i]);
        }
    } else {
        switch (state.ext_type) {
            case CWIID_EXT_NONE:
                printf("No extension\n");
                break;
            case CWIID_EXT_UNKNOWN:
                printf("Unknown extension attached\n");
                break;
            case CWIID_EXT_NUNCHUK:
                printf("Nunchuk\n");
                break;
            case CWIID_EXT_CLASSIC:
                printf("Classic");
                break;
            case CWIID_EXT_BALANCE:
                printf("Balance");
                break;
            case CWIID_EXT_MOTIONPLUS:
                printf("MotionPlus");
                break;
            default:
                printf("type=%x\n", state.ext_type);
        }
    }
    //set_disable(CWIID_FLAG_MOTIONPLUS);
}

void wiimote_c::updateWeightFromBlanceBoard() {
    struct balance_cal balance_cal;
    cwiid_get_balance_cal(body, &balance_cal);
    bbValues.leftTop = calcWeightFromBalanceBoard(state.ext.balance.left_top, balance_cal.left_top);
    bbValues.rightTop = calcWeightFromBalanceBoard(state.ext.balance.right_top, balance_cal.right_top);
    bbValues.leftBottom = calcWeightFromBalanceBoard(state.ext.balance.left_bottom, balance_cal.left_bottom);
    bbValues.rightBottom = calcWeightFromBalanceBoard(state.ext.balance.right_bottom, balance_cal.right_bottom);
}

//参考：https://github.com/derf/wii-sensors/blob/master/bal.c
float wiimote_c::calcWeightFromBalanceBoard(uint16_t reading, uint16_t cal[3]) {
    if (reading < cal[1])
        return ((float) reading - cal[0]) / (cal[1] - cal[0]) * 17.0;
    else
        return (((float) reading - cal[1]) / (cal[2] - cal[1]) * 17.0) + 17.0;
}

float wiimote_c::getTotalWeightFromBalanceBoard() {
    return bbValues.leftBottom + bbValues.leftTop + bbValues.rightBottom + bbValues.rightTop;
}

/*
 * バランスボードの重心(比率)をx,yそれぞれ返す．
 * 計算式としては，x>0において，y=-((1-x)/(1+x) - 1)
 * x<0においてはその逆．
 * x=0 の時に0で，x limit to 1で 1に漸近する．
 */
void wiimote_c::getPositionFromBalanceBoard(float position[2]) {
    position[0] = (bbValues.rightTop + bbValues.rightBottom) / (bbValues.leftTop + bbValues.leftBottom);
    if (position[0] > 1)
        position[0] = ((bbValues.leftTop + bbValues.leftBottom) / (bbValues.rightTop + bbValues.rightBottom) * (-1.0)) + 1.0;
    else
        position[0] -= 1;

    position[1] = (bbValues.leftTop + bbValues.rightTop) / (bbValues.leftBottom + bbValues.rightBottom);
    if (position[1] > 1)
        position[1] = ((bbValues.leftBottom + bbValues.rightBottom) / (bbValues.leftTop + bbValues.rightTop) * (-1.0)) + 1.0;
    else
        position[1] -= 1;

    //履歴の保存
    balanceXQueue.add(position[0]);
    balanceYQueue.add(position[1]);

}

void wiimote_c::getRevisedPositionFromBalanceBoard(float position[2]) {
    getPositionFromBalanceBoard(position);
    position[0]-=balanceXcriteria;
    position[1]-=balanceYcriteria;
}

void wiimote_c::initCriteriaOfBalanceBoard() {
    updateWeightFromBlanceBoard();
    balanceXcriteria = balanceXQueue.getAverage();
    balanceYcriteria = balanceYQueue.getAverage();
}
int wiimote_c::get_mesg(){
	int *mesg_count;
	union cwiid_mesg *mesg[15];
	struct timespec *timestamp;
	cwiid_get_mesg(body, mesg_count, mesg, timestamp);
	for (int i = 0; i < *mesg_count; i++) {
		switch (mesg[i]->type) {
//			case CWIID_MESG_STATUS:
//				printf("Status Report: battery=%d extension=", mesg[i].status_mesg.battery);
//				switch (mesg[i].status_mesg.ext_type) {
//					case CWIID_EXT_NONE:
//						printf("none");
//						break;
//					case CWIID_EXT_NUNCHUK:
//						printf("Nunchuk");
//						break;
//					case CWIID_EXT_CLASSIC:
//						printf("Classic Controller");
//						break;
//					case CWIID_EXT_BALANCE:
//						printf("Balance Board");
//						break;
//					case CWIID_EXT_MOTIONPLUS:
//						printf("MotionPlus");
//						break;
//					default:
//						printf("Unknown Extension");
//						break;
//				}
//				printf("\n");
//				break;
//			case CWIID_MESG_BTN:
//				printf("Button Report: %.4X\n", mesg[i].btn_mesg.buttons);
//				break;
//			case CWIID_MESG_ACC:
//				printf("Acc Report: x=%d, y=%d, z=%d\n",
//						mesg[i].acc_mesg.acc[CWIID_X],
//						mesg[i].acc_mesg.acc[CWIID_Y],
//						mesg[i].acc_mesg.acc[CWIID_Z]);
//				break;
//			case CWIID_MESG_IR:
//				printf("IR Report: ");
//				valid_source = 0;
//				for (j = 0; j < CWIID_IR_SRC_COUNT; j++) {
//					if (mesg[i].ir_mesg.src[j].valid) {
//						valid_source = 1;
//						printf("(%d,%d) ", mesg[i].ir_mesg.src[j].pos[CWIID_X],
//								mesg[i].ir_mesg.src[j].pos[CWIID_Y]);
//					}
//				}
//				if (!valid_source) {
//					printf("no sources detected");
//				}
//				printf("\n");
//				break;
//			case CWIID_MESG_NUNCHUK:
//				printf("Nunchuk Report: btns=%.2X stick=(%d,%d) acc.x=%d acc.y=%d "
//						"acc.z=%d\n", mesg[i].nunchuk_mesg.buttons,
//						mesg[i].nunchuk_mesg.stick[CWIID_X],
//						mesg[i].nunchuk_mesg.stick[CWIID_Y],
//						mesg[i].nunchuk_mesg.acc[CWIID_X],
//						mesg[i].nunchuk_mesg.acc[CWIID_Y],
//						mesg[i].nunchuk_mesg.acc[CWIID_Z]);
//				break;
//			case CWIID_MESG_CLASSIC:
//				printf("Classic Report: btns=%.4X l_stick=(%d,%d) r_stick=(%d,%d) "
//						"l=%d r=%d\n", mesg[i].classic_mesg.buttons,
//						mesg[i].classic_mesg.l_stick[CWIID_X],
//						mesg[i].classic_mesg.l_stick[CWIID_Y],
//						mesg[i].classic_mesg.r_stick[CWIID_X],
//						mesg[i].classic_mesg.r_stick[CWIID_Y],
//						mesg[i].classic_mesg.l, mesg[i].classic_mesg.r);
//				break;
//			case CWIID_MESG_BALANCE:
//				printf("Balance Report: right_top=%d right_bottom=%d "
//						"left_top=%d left_bottom=%d\n",
//						mesg[i].balance_mesg.right_top,
//						mesg[i].balance_mesg.right_bottom,
//						mesg[i].balance_mesg.left_top,
//						mesg[i].balance_mesg.left_bottom);
//				break;
//			case CWIID_MESG_MOTIONPLUS:
//				printf("MotionPlus Report: angle_rate=(%d,%d,%d) low_speed=(%d,%d,%d)\n",
//						mesg[i].motionplus_mesg.angle_rate[0],
//						mesg[i].motionplus_mesg.angle_rate[1],
//						mesg[i].motionplus_mesg.angle_rate[2],
//						mesg[i].motionplus_mesg.low_speed[0],
//						mesg[i].motionplus_mesg.low_speed[1],
//						mesg[i].motionplus_mesg.low_speed[2]);
//				break;
			case CWIID_MESG_ERROR:
				printf("Error Occures!!\r\n");
//				if (cwiid_close(wiimote)) {
//					fprintf(stderr, "Error on wiimote disconnect\n");
//					exit(-1);
//				}
//				exit(0);
				break;
			default:
				printf("Unknown Report");
				break;
		}
	}	
	return 0;
}
