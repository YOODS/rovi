#pragma once

typedef union tagUART_DATA_FIELD {
	uint32_t	dwData;
	struct {
		uint16_t	data;
		uint16_t	data_lrc;
	} w;
	struct {
		uint8_t		data;
		uint8_t 	null;
		uint8_t 	data_lrc;
		uint8_t 	null_lrc;
	} b;
	tagUART_DATA_FIELD() : dwData(0){}
} UART_DATA_FIELD;

#define REG_YCAM_VERSION		0x00000000
#define REG_IP_ADDRESS	        0x00000024
#define REG_PWM_FRAME_RATE		0x00101000
#define REG_EXPOSURE_TIME		0x00101004
#define REG_ANALOG_GAIN			0x00101008
#define REG_DIGITAL_GAIN		0x0010100C
#define REG_EXTERNAL_TRIGGER	0x00101010
#define REG_STREAM_NUM			0x00100020	//取得枚数 0:INFINITE
#define REG_UART				0x00300004
#define REG_TRANSFER_MODE		0x00101014
#define REG_CLOCK_DELAY			0x00101100
#define REG_FW_VERSION			0x00101110
#define REG_HEAT_BEAT_TIMEOUT	0x00000938

#define VER_ACAP		20		//このversionから非同期撮影を行う
#define PHSFT_CAP_NUM	13		//位相シフト撮影回数
//2020/11/06 add by hato ------------------ start ------------------
#define PHSFT3_CAP_NUM	14		//位相シフト(3位相)撮影回数
//2020/11/06 add by hato ------------------  end  ------------------

typedef union IPADDR{
	uint32_t a;
	uint8_t b[4];
	IPADDR(){ a = 0; }
} IPADDR;

//画像解像度
enum YCAM_RES {
	YCAM_RES_SXGA,	//2560x1024
	YCAM_RES_VGA	//1280x480
};

//左右
enum YCAM_SIDE{
	YCAM_SIDE_LEFT,	//左
	YCAM_SIDE_RIGHT	//右
};

/**
* @brief トリガーモード
*/
enum YCAM_TRIG
{
	YCAM_TRIG_INT,	//内部トリガ
	YCAM_TRIG_EXT	//外部トリガ
};

/**
* @brief PROJ発光パターン
*/
enum YCAM_PROJ_PTN
{
	YCAM_PROJ_PTN_FIXED,	//固定パターン
	YCAM_PROJ_PTN_PHSFT,	//位相シフト用
	YCAM_PROJ_PTN_STROBE,	//ストロボ撮影用
	YCAM_PROJ_PTN_FOCUS,	//ピント合わせ用
	YCAM_PROJ_PTN_STEREO,	//相関用
//2020/11/19 modified by hato ------------------ start ------------------
	YCAM_PROJ_PTN_PHSFT_3,	//位相シフト(3位相パターン 14枚)
//2020/11/19 modified by hato ------------------  end  ------------------
	nYCAM_PROJ_PTN			//count
};

/**
* @brief PROJ発光モード
*/
enum YCAM_PROJ_MODE
{
	YCAM_PROJ_MODE_ONCE,	//シングル発光(キャプチャなし / X枚目のパタンを照射; X = プロジェクター固定パターン)
	YCAM_PROJ_MODE_CAPT,	//シングル発光(キャプチャあり / 1枚目のパタンを照射)
	YCAM_PROJ_MODE_CONT,	//連続発光(キャプチャあり / 1から規定数のパタンを連続照射)
	nYCAM_PROJ_MODE			//count
};
