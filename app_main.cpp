#include "MPU9250.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "MPU9250.h"


extern "C" {
    void app_main(void);
}


void app_main()
{

    //Az itt található kikommentezett blokkok segítségével egyszerűen tesztelhetőek a megírt függvények


    //ALAP KONSTRUKTOR, SPI kommunikáció felállítása, esetleges eltérő PIN kiosztások megadása
    //MPU9250(long clock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ)
    MPU9250 senzor(32,14,0x11,0x11);
    senzor.spiinitialize();
    

    // // // /*     ALAP FGV-ek TESZTELÉSE
    //senzor.initialize_acc_and_gyro(BITSFS_4G, BITSFS_250, true,true);
    // senzor.whoami();
    // senzor.WriteReg(0x1A,0x0A); 
    // senzor.WriteReg(0x1C,0x10); 
    // senzor.ReadReg(0x1A,0x2A); 
    // senzor.ReadReg(0x1C,0X00); 
    // uint8_t RBuf[3];
    // senzor.ReadRegs(0x1A, RBuf,3);
    // printf("%d\n%d\n%d\n",RBuf[0],RBuf[1],RBuf[2]);
    // // // */
       

    // // /*        GYRO TESZTELÉS
    // Gyro_Scale scale2=BITSFS_1000; 
    // senzor.set_gyro_scale(scale2); 
    // senzor.calib_gyro(2,0,-10); 
    // senzor.auto_calib_gyro(); 
    // while(1)
    // { 
    //     const TickType_t delay= 50/portTICK_PERIOD_MS;
    //     vTaskDelay(delay);
    //     senzor.read_gyro();
    //     printf("%f,%f,%f\n", senzor.gyro_data[0],senzor.gyro_data[1],senzor.gyro_data[2]);
    // }
    // // */


    // // /*     ACC FGV TESZTELÉS
    // Accelometer_Scale scale=BITSFS_4G;
    // senzor.set_acc_scale(scale);
    // senzor.auto_calib_acc();
    // //senzor.calib_acc(0,-10,4);
    //  const TickType_t delay2= 1000/portTICK_PERIOD_MS;
    //     vTaskDelay(delay2);
    // while(1)
    // {
    //     const TickType_t delay= 50/portTICK_PERIOD_MS;
    //     vTaskDelay(delay);
    //     senzor.read_acc();
    //    printf("%f,%f,%f\n", senzor.accel_data[0],senzor.accel_data[1],senzor.accel_data[2]);
        
    // }
    // // */
    
   // // // /*     MIND A HÁRMAT ÉRINTŐ FÜGGVÉNYEK TESZTELÉSE
    senzor.init(true,true);
    senzor.auto_calib_mag();
        while(1)
        {
            senzor.read_all();
            printf(" %f,%f,%f,", senzor.accel_data[0],senzor.accel_data[1],senzor.accel_data[2]);
            printf(" %f,%f,%f,", senzor.gyro_data[0],senzor.gyro_data[1],senzor.gyro_data[2]);
            printf(" %f,%f,%f\n", senzor.mag_data[0],senzor.mag_data[1],senzor.mag_data[2]);
        }
    // // // */


    // // // /*     MAG FGV TESZTELÉS
    // senzor.set_mag_scale(BITSFS_16); 
    // senzor.init(true,true);
    // senzor.AK8963_whoami(); 
    // senzor.get_CNTL1(); 
    //senzor.calib_mag();
    //senzor.calib_offs_mag(12,600);
    // senzor.auto_calib_mag();
    //  while(1)
    // {
    //     const TickType_t delay= 50/portTICK_PERIOD_MS;
    //     vTaskDelay(delay);
    //     senzor.read_mag(); 
    //    printf("%f,%f,%f\n", senzor.mag_data[0],senzor.mag_data[1],senzor.mag_data[2]);  
    // }

    // // /*     LÁGYMÉGNESES ZAVAR ELLEN TESZT
    //senzor.calib_offs_mag(-20,-17.5,46);
    // senzor.auto_calib_mag();
    // while(1)
    // {
    //      const TickType_t delay= 50/portTICK_PERIOD_MS;
    //      vTaskDelay(delay);
    //      senzor.read_mag(); 
    //     senzor.calib_soft_iron_mag(25.22856, -15.36725, 11.25835, 16.02465);
    //     printf("%f,%f,%f\n", senzor.mag_data[0],senzor.mag_data[1],senzor.mag_data[2]);
    // }
    // // // */
}
