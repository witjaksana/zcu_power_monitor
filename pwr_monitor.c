/** pwr_monitor.c -- a program to monitor the power consumption on ZCU102 board
  * Copyright (C) 2018  Arief Wicaksana
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>. */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include "i2c-dev.h"

     //I2C channel
#define I2C_CHANNEL "/dev/i2c-2"
#define I2C_CHANNEL_PL "/dev/i2c-3"
     //log ifle
#define LOGFILE "./log.csv"
     //logging & printing variables
     //logging every 1/10 second
#define LOG_PERIOD_US 100000
     //updating screen every second: 10 x log_period
#define PRINT_PERIOD_FACTOR 10

     //logging flag for rails
#define LOG_RAIL_FALSE 0x00
#define LOG_RAIL_TRUE 0x01

     //PMBUS Commands
#define CMD_PAGE      0x00

#define DEFAULT_ITER_NB 1000

#define REG_CONFIG  0x00
#define REG_SHUNT_V 0x01
#define REG_BUS_V   0x02
#define REG_POWER   0x03
#define REG_CURRENT 0x04
#define REG_CAL     0x05
#define REG_EN      0x06
#define REG_ALERT   0x07
#define REG_ID      0xFE
#define REG_DIE     0xFF

     static volatile int stop = 0;

     struct voltage_rail {
         char *name;
         unsigned char device;
         double average_current;
         double average_power;
         unsigned char log;
     };

// Full Power Domain
struct voltage_rail zcu102_fp_rails[]={
    {
name	: "vccpsintfp       ",
          device	: 0x40,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "mgtravcc         ",
          device	: 0x44,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "mgtravtt         ",
          device	: 0x45,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vcco_psddr_504   ",
          device	: 0x46,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccpsddrpll      ",
          device	: 0x4b,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
};
// Low Power Domain
struct voltage_rail zcu102_lp_rails[]={
    {
name	: "vccpsintlp       ",
          device	: 0x41,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccpsaux         ",
          device	: 0x42,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccpspll         ",
          device	: 0x43,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccops           ",
          device	: 0x47,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccops3          ",
          device	: 0x4a,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
};
// Prog Logic Domain
struct voltage_rail zcu102_pl_rails[]={
    {
name	: "vccint           ",
          device	: 0x40,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccbram          ",
          device	: 0x41,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vccaux           ",
          device	: 0x42,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vcc1v2           ",
          device	: 0x43,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vcc3v3           ",
          device	: 0x44,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "vadj_fmc         ",
          device	: 0x45,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "mgtavcc          ",
          device	: 0x46,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
    {
name	: "mgtavtt          ",
          device	: 0x47,
          log	: LOG_RAIL_TRUE,
          average_current : 0.0,
          average_power   : 0.0 
    },
};

//interruption handler
void intHandler(int dummy) {
    printf("got ctrl-c, terminating loop\n");
    stop = 1;
}

void writeData(int fdi2c, unsigned char address, unsigned char reg, int value){
    int status;

    if (ioctl(fdi2c, I2C_SLAVE_FORCE, address) < 0){
        printf("ERROR: Unable to set I2C slave address 0x%02X\n", address);
        exit(1);
    }

    status = i2c_smbus_write_byte_data(fdi2c, CMD_PAGE, address);
    if (status < 0) {
        printf("ERROR: Unable to write page address to I2C slave at 0x%02X: %d\n", address, status);
        exit(1);
    }

    value = (value >> 8) | ((value & 0xff) << 8); /** turn the byte around */

    status = i2c_smbus_write_word_data(fdi2c, reg, value);
    if (status < 0) {
        printf("ERROR: Unable to write value to I2C reg at 0x%02X: %d\n", reg, status);
        exit(1);
    }
}

int readData(int fdi2c, unsigned char address, unsigned char reg){
    int status;
    int value;

    if (ioctl(fdi2c, I2C_SLAVE_FORCE, address) < 0){
        printf("ERROR: Unable to set I2C slave address 0x%02X\n", address);
        exit(1);
    }

    status = i2c_smbus_write_byte_data(fdi2c, CMD_PAGE, address);
    if (status < 0) {
        printf("ERROR: Unable to write page address to I2C slave at 0x%02X: %d\n", address, status);
        exit(1);
    }

    value = i2c_smbus_read_word_data(fdi2c, reg);
    value = (value >> 8) | ((value & 0xff) << 8); /** turn the byte around */
    return value;
}

double readBusVoltage(int fdi2c, unsigned char address){
    int raw_value;
    double voltage;

    raw_value = readData(fdi2c, address, REG_BUS_V);

    voltage = (float)raw_value * 0.00125;
    return voltage;
}

double readCurrent(int fdi2c, unsigned char address){
    int raw_value;
    double current;

    raw_value = readData(fdi2c, address, REG_CURRENT);
    // in case it's negative
    if ((raw_value & 0x8000) != 0){
        raw_value |= 0xffff0000;
    }

    current = (float)raw_value;
    return current;
}

double readPower(int fdi2c, unsigned char address){
    int raw_value;
    double power;

    raw_value = readData(fdi2c, address, REG_POWER);

    power = (float)raw_value * 0.025;
    return power;
}

void initCalibRail(int fdi2c, struct voltage_rail rail[], int j){
    // Current_LSB is 1mA/bit
    // R_shunt = 5 mili ohm
    int cal = 1024;
    int i;

    //printf("loop %d\n",j);
    for (i = 0; i < j; i++){
        writeData(fdi2c,rail[i].device,REG_CAL,cal); 
    }
}

int main(int argc, char *argv[]) {
    //testing parameters
    int iter_nb = DEFAULT_ITER_NB;
    if(argc == 2) iter_nb = atoi(argv[1]);

    //signal handling
    signal(SIGINT, intHandler);

    // file descriptor for i2c bus
    int fdi2c;
    int fdi2c_pl;
    FILE* logfile;

    int i, h;
    int loop_fp, loop_lp, loop_pl;
    double totalPower;
    double power;
    double fp_power, lp_power, pl_power;
    double voltage;
    double current;

    double maxpower = 0.0f;

    //opening log file
    logfile = fopen(LOGFILE,"w");
    if(logfile < 0)
    {
        printf("Cannot create logfile\n");
        return 1;
    }
    fprintf(logfile,"logging file from pmbus_test. Columns : iter_nb ; voltage (V), current (mA), power (mW).\n");

    //opening device for read and write
    fdi2c = open(I2C_CHANNEL, O_RDWR);
    if(fdi2c < 0)
    {
        printf("Cannot open the device\n");
        return 1;
    }
    fdi2c_pl = open(I2C_CHANNEL_PL, O_RDWR);
    if(fdi2c_pl < 0)
    {
        printf("Cannot open the PL device\n");
        return 1;
    }

    loop_fp = sizeof(zcu102_fp_rails)/sizeof(struct voltage_rail);
    loop_lp = sizeof(zcu102_lp_rails)/sizeof(struct voltage_rail);
    loop_pl = sizeof(zcu102_pl_rails)/sizeof(struct voltage_rail);

    // Initialization
    initCalibRail(fdi2c, zcu102_fp_rails, loop_fp); // init full power rails
    initCalibRail(fdi2c, zcu102_lp_rails, loop_lp); // init low power rails
    initCalibRail(fdi2c_pl, zcu102_pl_rails, loop_pl); // init logic power rails

    h = 0;
    while((stop == 0) && (h < iter_nb)){
        totalPower = 0.0f;
        fp_power = 0.0f;
        lp_power = 0.0f;
        pl_power = 0.0f;
        power = 0.0f;
        for(i = 0; i < loop_fp; i++) {
            zcu102_fp_rails[i].average_power = 0;
            zcu102_fp_rails[i].average_current=0;
        } 
        for(i = 0; i < loop_lp; i++) {
            zcu102_lp_rails[i].average_power = 0;
            zcu102_lp_rails[i].average_current=0;
        } 
        for(i = 0; i < loop_pl; i++) {
            zcu102_pl_rails[i].average_power = 0;
            zcu102_pl_rails[i].average_current=0;
        } 
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("\033[?1049h\033[H");
            printf("+-------------------------------------------------------------------------------+\n");
            printf("|                                     Power Monitor                             |\n");
            printf("|RAIL                         |   Voltage(V)    |   Current(mA) |    Power(mW)  |\n");
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("+-------------------------------------------------------------------------------+\n");
            printf("|                             Full Power Domain                                 |\n");
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
        }
        for(i = 0; i < loop_fp; i++) {
            voltage = readBusVoltage(fdi2c, zcu102_fp_rails[i].device);
            current = readCurrent(fdi2c, zcu102_fp_rails[i].device);
            power = voltage * current;
            fp_power += power;
            totalPower += power;
            zcu102_fp_rails[i].average_current = current;
            zcu102_fp_rails[i].average_power = power;
            if(h%PRINT_PERIOD_FACTOR == 0) {
                printf("|%s \t      | %.5f V\t| %.1f mA\t| %.4f mW\t|\n",zcu102_fp_rails[i].name,voltage,current,power);
            }
            //logging only rails that have the log flag
            if(zcu102_fp_rails[i].log == LOG_RAIL_TRUE){
                fprintf(logfile,"%d, %.5f, %.1f, %f\n", h, voltage, current, power);
            }
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
            printf("|TOTAL FULL POWER   %f mW\t                                        |\n",fp_power);
            printf("+-----------------------------------------------+-------------------------------+\n");
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("|                               Low Power Domain                                |\n");
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
        }
        for(i = 0; i < loop_lp; i++) {
            voltage = readBusVoltage(fdi2c, zcu102_lp_rails[i].device);
            current = readCurrent(fdi2c, zcu102_lp_rails[i].device);
            power = voltage * current;
            lp_power += power;
            totalPower += power;
            zcu102_lp_rails[i].average_current = current;
            zcu102_lp_rails[i].average_power = power;
            if(h%PRINT_PERIOD_FACTOR == 0){
                printf("|%s \t      | %.5f V\t| %.1f mA\t| %.4f mW\t|\n",zcu102_lp_rails[i].name,voltage,current,power);
            }
            //logging only rails that have the log flag
            if(zcu102_lp_rails[i].log == LOG_RAIL_TRUE){
                fprintf(logfile,"%d, %.5f, %.1f, %f\n", h, voltage, current, power);
            }
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
            printf("|TOTAL LOW POWER    %f mW\t                                        |\n",lp_power);
            printf("+-----------------------------------------------+-------------------------------+\n");
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("|                           Prog Logic Power Domain                             |\n");
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
        }
        for(i = 0; i < loop_pl; i++) {
            voltage = readBusVoltage(fdi2c_pl, zcu102_pl_rails[i].device);
            current = readCurrent(fdi2c_pl, zcu102_pl_rails[i].device);
            power = voltage * current;
            pl_power += power;
            totalPower += power;
            zcu102_pl_rails[i].average_current = current;
            zcu102_pl_rails[i].average_power = power;
            if(h%PRINT_PERIOD_FACTOR == 0){
                printf("|%s \t      | %.5f V\t| %.1f mA\t| %.4f mW\t|\n",zcu102_pl_rails[i].name,voltage,current,power);
            }
            //logging only rails that have the log flag
            if(zcu102_pl_rails[i].log == LOG_RAIL_TRUE){
                fprintf(logfile,"%d, %.5f, %.1f, %f\n", h, voltage, current, power);
            }
        }
        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("+-----------------------------+-----------------+---------------+---------------+\n");
            printf("|TOTAL PROG LOGIC POWER     %f mW\t                                |\n",pl_power);
            printf("+-----------------------------------------------+-------------------------------+\n");
        }

        if (maxpower < totalPower ) maxpower= totalPower;

        if(h%PRINT_PERIOD_FACTOR == 0){
            printf("|TOTAL POWER %f mW\t                | MAX POWER  %f mW\t|\n",totalPower,maxpower);
            printf("+-----------------------------------------------+-------------------------------+\n");
        }

        fflush(stdout);
        usleep(LOG_PERIOD_US);
        h++;
    }

    close(fdi2c_pl);
    close(fdi2c);
    printf("ending -- everything ok\n");
    return 0;
}
