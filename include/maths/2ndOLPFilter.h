#ifndef _2NDOLPFILTER_H_
#define _2NDOLPFILTER_H_

#ifdef __cplusplus
	extern "C"{
#endif
		
typedef struct{
	float E0,E1,E2;
	float a1,a2;
	float b0,b1,b2;
	int cutofffreq;
}
FILTER_BUFFUER;

typedef struct{
	int len;
	unsigned char buf[200];
}
FILTER_QUEUE;   

void filter_init(float sample, int samplefreq, int cutofffreq, FILTER_BUFFUER* _filter);
void filter_update(float sample, FILTER_BUFFUER* _filter);
float filter_output(FILTER_BUFFUER* _filter);
		
#ifdef __cplusplus
	}
#endif

#endif