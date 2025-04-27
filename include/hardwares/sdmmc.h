#ifndef _SDMMC_H_
#define _SDMMC_H_

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

#define SDCMD_OK										0
#define SDCMD_TIMEOUT								1
#define SDCMD_CRCERROR							1<<1
#define SDCMD_CMDERROR							1<<2

#define SD_CARDSTATE_ERRORBITS      0xFDFFE008

//SD4.0 cmd class 0
#define SDCMD_GO_IDLE_STATE					0
#define SDCMD_ALL_SEND_CID					2
#define SDCMD_SEND_RELATIVE_ADDR		3
#define SDCMD_SET_DSR								4
#define SDCMD_SELECT_DESELECT_CARD	7
#define SDCMD_IF_COND								8
#define SDCMD_CSD										9
#define SDCMD_CID										10
#define SDCMD_VOLTAGE_SWITCH				11
#define SDCMD_STOP_TRANSMISSION			12
#define SDCMD_SEND_STATUS						13
#define	SDCMD_GO_INACTIVE_STATE			15
//SD4.0 cmd class 2&4
#define SDCMD_SET_BLOCKLEN					16
#define SDCMD_READ_SINGLE_BLOCK			17
#define SDCMD_READ_MULTIPLE_BLOCK		18
#define SDCMD_SEND_TUNING_BLOCK			19
#define SDCMD_SPEED_CLASS_CONTROL		20
#define SDCMD_SET_BLOCK_COUNT				23
#define SDCMD_WRITE_BLOCK						24
#define SDCMD_WRITE_MULTIPLE_BLOCK	25
#define SDCMD_PROGRAM_CSD						27
//SD4.0 cmd class 6
#define SDCMD_SET_WRITE_PORT				28
#define SDCMD_CLEAR_WRITE_PORT			29
#define SDCMD_SED_WRITE_PORT				30
//SD4.0 cmd class 5
#define SDCMD_ERASE_WR_BLK_START		32
#define SDCMD_ERASE_WR_BLK_END			33
#define SDCMD_ERASE									38
//SD4.0 cmd class 8
#define SDCMD_APP_CMD								55
#define SDCMD_GEN_CMD								56
//SD4.0 acmd
#define SDACMD_SET_BUS_WIDTH				6
#define SDACMD_SEND_OP_COND					41
#define SDACMD_SEND_SCR							51


#ifdef __cplusplus
	extern "C"{
#endif
		typedef
		struct{
			union{
				unsigned long CID[4];
				struct{
					unsigned res	:	1;
					unsigned IDCRC:	7;
					unsigned MDT	:	12;
					unsigned res1	:	4;
					unsigned PSN	:	32;
					unsigned PRV	:	8;
					unsigned PNM0	:	20;
					unsigned PNM1	:	20;
					unsigned OID	:	16;
					unsigned MID	:	8;
				};
			};
			union{
				unsigned long CSD[4];
				struct{
					unsigned res2								:	1;
					unsigned SDCRC							:	7;
					unsigned res3								:	2;
					unsigned FILE_FORMAT				: 2;
					unsigned TMP_WRITE_PROTECT	:	1;
					unsigned PERM_WRITE_PROTECT	:	1;
					unsigned COPY								:	1;
					unsigned FILE_FORMAT_GRP		:	1;
					unsigned res4								:	5;
					unsigned WRITE_BL_PARTAIL		:	1;
					unsigned WRITE_BL_LEN				:	4;
					unsigned R2W_FACTOR					:	3;
					unsigned res5								:	2;
					unsigned WP_GRP_ENALBE			:	1;
					unsigned WP_GRP_SIZE				:	7;
					unsigned SECTOR_SIZE				:	7;
					unsigned EEASE_BLK_EN				:	1;
					unsigned res6								:	1;
					unsigned C_SIZE							:	22;
					unsigned res7								:	6;
					unsigned DSR_IMP						:	1;
					unsigned READ_BLK_MISALIGN	:	1;
					unsigned WRITE_BLK_MISALIGN	:	1;
					unsigned READ_BL_PARTIAL		:	1;
					unsigned READ_BL_LEN				:	4;
					unsigned CCC								:	12;
					unsigned TRAN_SPEED					:	8;
					unsigned NSAC								:	8;
					unsigned TAAC								:	8;
					unsigned res8								:	6;
					unsigned CSD_STRUCTURE			:	2;
				};
			};
			union{
				unsigned long SCR[2];
				struct{
					unsigned res9								:	32;
					unsigned CMD_SUPPORT				:	2;
					unsigned res10							:	9;
					unsigned EX_SECURITY				:	4;
					unsigned SD_SPEC3						:	1;
					unsigned SD_BUS_WIDTHS			:	4;
					unsigned SD_SECURITY				:	3;
					unsigned DATA_STAT_AFTER_ERASE:1;
					unsigned SD_SPEC						:	4;
					unsigned SCR_STRUCTURE			:	4;
				};
			};
			unsigned short RCA;
			unsigned char Type;
		}
		SDCARD_INFO_DEF;
		
		int sdmmc_sendCmd(unsigned char cmd,unsigned char wait,unsigned long arg);
		
		extern SDCARD_INFO_DEF sdcard_info;
		
		int sdcard_init(void);
		
		int SD_ReadDisk(unsigned char *buf,unsigned long addr,unsigned long num);
		
		int SD_WriteDisk(unsigned char *buf,unsigned long addr,unsigned long num);
		
#ifdef __cplusplus
	}
#endif

#endif