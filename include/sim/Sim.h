#ifndef _SIM_H_
#define _SIM_H_

#ifdef __cplusplus
extern "C" {
#endif

    typedef unsigned char BYTE;
    typedef unsigned short WORD;
    typedef unsigned long DWORD;
    typedef unsigned int UINT;

#define	Rad2Deg 	57.2957795130823208767981548141052

    void ctrl_flytask(void);
    void ctrl_level(void);
    void ctrl_rectangular(void);
    void ctrl_approach(void);
    void ctrl_cmdSmooth(void);
    void simu_init(void);

#ifdef __cplusplus
}
#endif   
#endif