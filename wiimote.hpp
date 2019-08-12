/* 
 * File:   wiimote.hpp
 * Author: ekrixis
 *
 * Created on 2015/07/29, 11:20
 */

#ifndef WIIMOTE_HPP
#define	WIIMOTE_HPP

#include <cwiid.h>//ホントは構造体を前方参照とかで宣言して，ヘッダファイル内のincludeは避けるべきらしいが，あまりに面倒なので省略

#define toggle_bit(bf,b)	\
	(bf) = ((bf) & b)		\
	       ? ((bf) & ~(b))	\
	       : ((bf) | (b))

struct BalanceBoardValue{
    float leftTop,leftBottom,rightTop,rightBottom;
};
//過去N個のfloatの平均をとりたいだけのクラス．個数<nの場合は考えないこととする．
class LightFloatQueue{
    static const int MAX=10;
    float value[MAX];
    int cnt;
public:
    LightFloatQueue(){
    };
    void add(float val){
	value[cnt]=val;
	cnt = cnt<(MAX-1) ? cnt+1:0;
    }
    float getAverage(){
	float ret=0.;
	for(int i=0;i<MAX;i++){
	    ret+=value[i];
	}
	return ret/MAX;
    }
};
class wiimote_c//callbackは設定できないっぽい？
{
private:
	cwiid_wiimote_t *body;
	bdaddr_t bdaddr;
	LightFloatQueue balanceXQueue,balanceYQueue;
	float balanceXcriteria,balanceYcriteria;
	
	float calcWeightFromBalanceBoard(uint16_t reading, uint16_t cal[3]);
public:
        struct cwiid_state state;
	struct BalanceBoardValue bbValues;
	
        unsigned char rpt_mode;
        int16_t angle_rates[3];
	
        wiimote_c();
	
	int open();
	int open(int timeout);
	int close();
	int toggle_led(unsigned char led_no);
	int set_rpt_mode(unsigned char lcl_rpt_mode);
	int clr_rpt_mode(unsigned char lcl_rpt_mode);
	void set_enable(int flags);
	void set_disable(int flags);
	int get_state();
	void get_motionplus_state();
	void updateWeightFromBlanceBoard();
	float getTotalWeightFromBalanceBoard();
	void initCriteriaOfBalanceBoard();
	void getPositionFromBalanceBoard(float position[2]);
	void getRevisedPositionFromBalanceBoard(float position[2]); 
	void set_err(cwiid_err_t err);
	int get_mesg();
};

#endif	/* WIIMOTE_HPP */

