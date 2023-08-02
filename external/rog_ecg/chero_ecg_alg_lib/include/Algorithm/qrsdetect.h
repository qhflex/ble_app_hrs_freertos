//*****************************************************************************
//FILE:  qrsdetect.h
//*****************************************************************************

#ifndef _QRSDET_H
#define _QRSDET_H



	int QRSDetect(int datum, int qrsIndex, int *winPeak, int *filterData, int* sbPeak, int*hr);
	int resetQRSDetect(int minRrInterval);


#endif 