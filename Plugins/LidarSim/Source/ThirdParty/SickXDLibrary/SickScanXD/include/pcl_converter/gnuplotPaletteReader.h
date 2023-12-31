#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef SICK_SCAN_GNU_PLOT_PALETTE_H
#define SICK_SCAN_GNU_PLOT_PALETTE_H

class GnuPlotPalette
{
public:
	GnuPlotPalette();
	unsigned char getRbgValue(unsigned char greyIdx, unsigned int channelIdx);
	int load(std::string palettName);
	void setErrorStatus(int _errorCode)
	{
		errorStatus = _errorCode;
	}

	int getErrorStatus(void)
	{
		return(errorStatus);
	}

private:
	unsigned char rgbTable[256][3];  // r g b - table entries
	int errorStatus;
};
#endif