#ifndef LIDAR_H
#define LIDAR_H

#include <I2C.h>

#define LIDARLITE_ADDR_DEFAULT 0x62

class LIDAR
{
  public:
	  I2C *lidar;
      LIDAR(int configuration, char lidarAddress);
      void configure(int = 0, char = LIDARLITE_ADDR_DEFAULT);
      void reset(char = LIDARLITE_ADDR_DEFAULT);
      int distance(bool = true, char = LIDARLITE_ADDR_DEFAULT);
      void write(char, char);
      void read(char, int,unsigned char*);
      void correlationRecordToSerial(char = '\n', int = 256, char = LIDARLITE_ADDR_DEFAULT);
};

#endif
