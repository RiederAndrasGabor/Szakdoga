#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h" 

#ifndef MPU9250_h
#define MPU9250_h



// mpu9250 szenzor regszterei:
//Több, mint amennyit ténylegesen használ a program, de nem az összes
#define MPUREG_XG_OFFS_TC 0x00
#define MPUREG_YG_OFFS_TC 0x01
#define MPUREG_ZG_OFFS_TC 0x02
#define MPUREG_X_FINE_GAIN 0x03
#define MPUREG_Y_FINE_GAIN 0x04
#define MPUREG_Z_FINE_GAIN 0x05
#define MPUREG_XA_OFFS_H 0x06
#define MPUREG_XA_OFFS_L 0x07
#define MPUREG_YA_OFFS_H 0x08
#define MPUREG_YA_OFFS_L 0x09
#define MPUREG_ZA_OFFS_H 0x0A
#define MPUREG_ZA_OFFS_L 0x0B
#define MPUREG_PRODUCT_ID 0x0C
#define MPUREG_SELF_TEST_X 0x0D
#define MPUREG_SELF_TEST_Y 0x0E
#define MPUREG_SELF_TEST_Z 0x0F
#define MPUREG_SELF_TEST_A 0x10
#define MPUREG_XG_OFFS_USRH 0x13
#define MPUREG_XG_OFFS_USRL 0x14
#define MPUREG_YG_OFFS_USRH 0x15
#define MPUREG_YG_OFFS_USRL 0x16
#define MPUREG_ZG_OFFS_USRH 0x17
#define MPUREG_ZG_OFFS_USRL 0x18
#define MPUREG_SMPLRT_DIV 0x19
#define MPUREG_CONFIG 0x1A
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_ACCEL_CONFIG_2      0x1D
#define MPUREG_LP_ACCEL_ODR        0x1E
#define MPUREG_MOT_THR             0x1F
#define MPUREG_FIFO_EN             0x23
#define MPUREG_I2C_MST_CTRL        0x24
#define MPUREG_I2C_SLV0_ADDR       0x25
#define MPUREG_I2C_SLV0_REG        0x26
#define MPUREG_I2C_SLV0_CTRL       0x27
#define MPUREG_I2C_SLV1_ADDR       0x28
#define MPUREG_I2C_SLV1_REG        0x29
#define MPUREG_I2C_SLV1_CTRL       0x2A
#define MPUREG_I2C_SLV2_ADDR       0x2B
#define MPUREG_I2C_SLV2_REG        0x2C
#define MPUREG_I2C_SLV2_CTRL       0x2D
#define MPUREG_I2C_SLV3_ADDR       0x2E
#define MPUREG_I2C_SLV3_REG        0x2F
#define MPUREG_I2C_SLV3_CTRL       0x30
#define MPUREG_I2C_SLV4_ADDR       0x31
#define MPUREG_I2C_SLV4_REG        0x32
#define MPUREG_I2C_SLV4_DO         0x33
#define MPUREG_I2C_SLV4_CTRL       0x34
#define MPUREG_I2C_SLV4_DI         0x35
#define MPUREG_I2C_MST_STATUS      0x36
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E
#define MPUREG_ACCEL_ZOUT_H 0x3F
#define MPUREG_ACCEL_ZOUT_L 0x40
#define MPUREG_GYRO_XOUT_H 0x43
#define MPUREG_GYRO_XOUT_L 0x44
#define MPUREG_GYRO_YOUT_H 0x45
#define MPUREG_GYRO_YOUT_L 0x46
#define MPUREG_GYRO_ZOUT_H 0x47
#define MPUREG_GYRO_ZOUT_L 0x48
#define MPUREG_EXT_SENS_DATA_00 0x49
#define MPUREG_EXT_SENS_DATA_01 0x50
#define MPUREG_EXT_SENS_DATA_02 0x51
#define MPUREG_EXT_SENS_DATA_03 0x52
#define MPUREG_EXT_SENS_DATA_04 0x53
#define MPUREG_EXT_SENS_DATA_05 0x54
#define MPUREG_EXT_SENS_DATA_06 0x55
#define MPUREG_EXT_SENS_DATA_07 0x56
#define MPUREG_EXT_SENS_DATA_08 0x57
#define MPUREG_EXT_SENS_DATA_09 0x58
#define MPUREG_EXT_SENS_DATA_10 0x59
#define MPUREG_EXT_SENS_DATA_11 0x60
#define MPUREG_EXT_SENS_DATA_12 0x61
#define MPUREG_EXT_SENS_DATA_13 0x62

#define MPUREG_I2C_SLV0_DO         0x63
#define MPUREG_I2C_SLV1_DO         0x64
#define MPUREG_I2C_SLV2_DO         0x65
#define MPUREG_I2C_SLV3_DO         0x66
#define MPUREG_I2C_MST_DELAY_CTRL  0x67
#define MPUREG_SIGNAL_PATH_RESET   0x68
#define MPUREG_MOT_DETECT_CTRL     0x69
#define MPUREG_USER_CTRL 0x6A
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_PWR_MGMT_2 0x6C
#define MPUREG_BANK_SEL 0x6D
#define MPUREG_MEM_START_ADDR 0x6E
#define MPUREG_MEM_R_W 0x6F
#define MPUREG_DMP_CFG_1 0x70
#define MPUREG_DMP_CFG_2 0x71
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_WHOAMI 0x75
#define MPUREG_XA_OFFSET_H         0x77
#define MPUREG_XA_OFFSET_L         0x78
#define MPUREG_YA_OFFSET_H         0x7A
#define MPUREG_YA_OFFSET_L         0x7B
#define MPUREG_ZA_OFFSET_H         0x7D
#define MPUREG_ZA_OFFSET_L         0x7E

/* ---- AK8963, magnetométeres regiszterek MPU9250-ben ----------------------------------------------- */
 
#define AK8963_I2C_ADDR             0x0c//0x18
#define AK8963_Device_ID            0x48//72, arduino-val megkapott cím
 
// Read-only regiszterek
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
// Write/Read regiszterek
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
// Read-only regiszterek ( ROM )
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
 
// mpu9250 konfigurációs bitek
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
 
#define READ_FLAG   0x80
 
/* ---- mérőtartomány beállításához szükséges  konstansok --------------------------------------------------------- */
 
#define MPU9250A_2g       ((float)0.000061035156f) // 0.000061035156 g/LSB
#define MPU9250A_4g       ((float)0.000122070312f) // 0.000122070312 g/LSB
#define MPU9250A_8g       ((float)0.000244140625f) // 0.000244140625 g/LSB
#define MPU9250A_16g      ((float)0.000488281250f) // 0.000488281250 g/LSB
 
#define MPU9250G_250dps   ((float)0.007633587786f) // 0.007633587786 dps/LSB
#define MPU9250G_500dps   ((float)0.015267175572f) // 0.015267175572 dps/LSB
#define MPU9250G_1000dps  ((float)0.030487804878f) // 0.030487804878 dps/LSB
#define MPU9250G_2000dps  ((float)0.060975609756f) // 0.060975609756 dps/LSB
 
#define MPU9250M_4800uT   ((float)0.6f)            // 0.6 uT/LSB , mag. 14 bites eseténél
  
#define     Magnetometer_Sensitivity_Scale_Factor ((float)0.15f)    //mag. 16 bites eseténél
 

/* ---A gyorsulásmérő 4 mérési tartományát ezekkel a bitekkel lehet beállítani ---*/
enum Accelometer_Scale {BITSFS_2G=0x00, BITSFS_4G=0x08, BITSFS_8G=0x10, BITSFS_16G=0x18};
/* ---A giroszkóp 4 mérési tartományát ezekkel a bitekkel lehet beállítani ---*/
enum Gyro_Scale {BITSFS_250=0x00, BITSFS_500=0x08, BITSFS_1000=0x10, BITSFS_2000=0x18};
/* ---A magnetométer 2 db mérési tartományát ezekkel a bitekkel lehet beállítani ---*/
enum Magneto_Scale {BITSFS_14=0, BITSFS_16=1};

class MPU9250 {   
    private:
    //Chip select
    uint8_t my_cs;

    //SPI-hez szükséges változó
    spi_device_handle_t spi_dev_mpu9250;

    // Offset hiha kompenzálása
    float g_bias[3]={0,0,0};
    float a_bias[3]={0,0,0};      
    float m_bias[3]={0,0,0};
public:

    // paraméterezéshez, illetve skálázáshoz szükséges változók
    float acc_divider;
    float gyro_divider;
    int calib_data[3];
    float Magnetometer_ASA[3];

    // 3*3 szögelfordulást, mágnesesség és gyorsulást tároló adatok
    float accel_data[3];
    float gyro_data[3];
    float mag_data[3];
    int16_t mag_data_raw[3];

    //Egy a legfontosabb paramétereket kezelő struktúra
    struct { 
        //Aluláteresztő szűrő hány Hz-es legyen
        uint8_t my_low_pass_filter;
        uint8_t my_low_pass_filter_acc;
        //Az órajel PIN-je
        long my_clock;

        //A gyorsulásmérő, a giroszkóp és a magnetométer mérési tartományai
        unsigned int a_scale=0;
        unsigned int g_scale=0;
        float mag_scale;

        // Itt követhető nyomon az automatikus offset kompenzálás megléte
        bool a_offset=false;
        bool g_offset=false;
        bool mag_offset=false;
    } parameters;


    // alap konstruktor, inicializálatlan esetben 188 Hz-es alul áteresztő szűrővel
    MPU9250(long mpuclock, uint8_t cs, uint8_t low_pass_filter = BITS_DLPF_CFG_188HZ, uint8_t low_pass_filter_acc = BITS_DLPF_CFG_188HZ){
        parameters.my_clock = mpuclock;
        my_cs = cs;
        parameters.my_low_pass_filter = low_pass_filter;
        parameters.my_low_pass_filter_acc = low_pass_filter_acc;
    }

    //SPI kommunikáció felállításáért felelős függvények
    void spiinitialize();
    void busconfig(int mosi_io_num = 25, int miso_io_num = 34, int sclk_io_num = 32,int quadwp_io_num = -1, int quadhd_io_num = -1, int max_transfer_sz = 32);
    void businitialize(uint8_t mode = 0, int clock_speed_hz = 500000, int spics_io_num = 14, uint32_t  flags = 0, int queue_size = 1, transaction_cb_t pre_cb = NULL, transaction_cb_t  post_cb = NULL);


    //Regiszterekhez hozzáférés
    unsigned int WriteReg(uint8_t WriteAddr, uint8_t WriteData );
    unsigned int ReadReg(uint8_t WriteAddr, uint8_t WriteData );
    void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes );

    //Gyorsulásmérő, és giroszkóp mérési tartománya, és automatikus offset kompenzálás megadása
    //"init" függvény segítségével életre kel a magnetométer is
    void initialize_acc_and_gyro(Accelometer_Scale acc_scale=BITSFS_2G, Gyro_Scale gyro_scale=BITSFS_250, bool calib_acc=false, bool calib_gyro=false);
    bool init(bool calib_gyro = true, bool calib_acc = true);

    //A három szenzor mérési tartományának skálázása
    unsigned int set_gyro_scale(Gyro_Scale scale);
    unsigned int set_acc_scale(Accelometer_Scale scale);
    float set_mag_scale(Magneto_Scale scale);

    //Alap regiszterek értékeinek olvasása, első tesztekhez hasznos
    unsigned int whoami(); 
    uint8_t AK8963_whoami();
    uint8_t get_CNTL1(); 

    //Szenzor vagy szenzorok értékeinek kiolvasása
    void read_acc();
    void read_gyro();
    void read_mag();
    void read_all();

    //Automatikus offset kompenzálások
    void auto_calib_acc();
    void auto_calib_gyro();
    void auto_calib_mag();

    //Kézzel történő offset megadása, 
    //Magnetométer skálázási tényezőjének meghívása, és lágymágneses zavar kompenzálása
    void calib_acc(float XA=0, float YA=0, float ZA=0);
    void calib_gyro(float XG=0, float YG=0, float ZG=0);
    void calib_mag();
    void calib_offs_mag(float XM=0, float YM=0, float ZM=0);
    void calib_soft_iron_mag(float X1=0, float Y1=0,float X2=0, float Y2=0);
    

};
 
#endif
