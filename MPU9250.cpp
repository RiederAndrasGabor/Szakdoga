#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "MPU9250.h"
#include <ctime>
#include "esp_err.h"
#include <math.h>

/**
 * \brief    Blokkoló késleltetést megvalósító függvény
 * Magnetométer használatakor kerül alkalmazásra
 */
int delay(int milliseconds) 
{
     clock_t goal = milliseconds + clock();
     while(goal>clock());
     return 1;
}


/**
 *  SPI inicializálását elvégző függvény
 *  meghívás esetén először beállítja a megfelelő használt portokat,
 * ezt követően pedig a kommunikáció konfigurálása következik
 */
void MPU9250::spiinitialize()
{
    // SPI busz inicializálása
    spi_bus_config_t buscfg={
        .mosi_io_num = 25,
        .miso_io_num = 34, 
        .sclk_io_num = 32,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
        };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
    //SPI konfigurációja
    spi_device_interface_config_t devcfg={
         .mode = 0,                  //SPI mode 0, alapértelmezetten 0 az órajel értéke, és felfutó élre történik mintavételezés
        .clock_speed_hz = 250000,  // 0,25 MHz, maximum 1 MHz lehet
        .spics_io_num = 14,         // CS Pin
          .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev_mpu9250));
    return;
}


/**
 *  A gyorsulásmérő és a giroszkóp inicializálását elvégző függvény
 *  Beállítható vele a mérési tartomány, és az automatikus offset kompenzálás kedv szerint.
 * Először a skálát állítjuk be, majd atán az offsetet, alapértelmezett esetben a 
 * legkisebb mérési tartomány kerül beállításra, és az offset kompenzálással nem számolunk.
 */
 void MPU9250::initialize_acc_and_gyro(Accelometer_Scale acc_scale, Gyro_Scale gyro_scale, bool calib_acc, bool calib_gyro)
 { 
    set_acc_scale(acc_scale);
    set_gyro_scale(gyro_scale);
    if(calib_gyro && calib_acc){
        auto_calib_acc();
        auto_calib_gyro();
    }
    else if(calib_gyro){
        auto_calib_gyro();
    }
    else if(calib_acc){
        auto_calib_acc();
    }
    return;
 }




/**
 * \brief    SPI busz konfigurációjának paraméterei itt állíthatóak
 */
void MPU9250::busconfig(int mosi_io_num, int miso_io_num, int sclk_io_num,int quadwp_io_num, int quadhd_io_num, int max_transfer_sz)
{
     spi_bus_config_t buscfg={
        .mosi_io_num = mosi_io_num,
        .miso_io_num = miso_io_num, 
        .sclk_io_num = sclk_io_num,
        .quadwp_io_num = quadwp_io_num,
        .quadhd_io_num = quadhd_io_num,
        .max_transfer_sz = max_transfer_sz,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, 0));
        return;
}

/**
 * \brief    SPI busz inicializálásának paraméterei itt állíthatóak 
 */
void MPU9250::businitialize(uint8_t mode, int clock_speed_hz, int spics_io_num, uint32_t  flags, int queue_size , transaction_cb_t pre_cb, transaction_cb_t  post_cb)
{
    spi_device_interface_config_t devcfg={
         .mode = mode,                  //SPI mode 0
        .clock_speed_hz = clock_speed_hz,  // 0,5 MHz
        .spics_io_num = spics_io_num,         // CS Pin
          .flags = flags,
        .queue_size = queue_size,
        .pre_cb = pre_cb,
        .post_cb = post_cb,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_dev_mpu9250));
}


/**                 regiszter írása
 *  
*  Meg kell adni a címet, amire írni szeretnénk, illetve az adatot.
 *  0-val kell kezdődnie az adatnak, a 2 bájtos adat a regiszterbe bekerül
 *  visszatérési érték: a regiszter tartalma (ellenőrzésképpen)
 */
unsigned int MPU9250::WriteReg( uint8_t WriteAddr, uint8_t WriteData )
{
    uint8_t a=0b01111111;
    WriteAddr=(WriteAddr&a); // Biztosítva a 0-ás kezdő bitet
    uint8_t tx_data[2] = { WriteAddr, WriteData};
    uint8_t rx_data[2] = { 0xFF, 0xFF};
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };  
    // kiírásra került az adat
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
    return(rx_data[1]);
}


/**                 regiszter olvasása
 *  
 *  Meg kell adni a címet, amiről olvasni szeretnénk, illetve az adatot
 *  1-gyel kell kezdődnie az adatnak, 2 bájtot olvasunk ki a megadott regiszterből
 *  visszatérési érték: a regiszter tartalma
 */
unsigned int  MPU9250::ReadReg( uint8_t WriteAddr, uint8_t WriteData )
{
    uint8_t a=0b10000000;
    WriteAddr=(WriteAddr|a); // Itt az 1-es kezdőbitről gondoskodunk 
    uint8_t tx_data[2] = { WriteAddr, WriteData};
    uint8_t rx_data[2] = { 0xFF, 0xFF};
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
    };  
    // kiírásra került az adat
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
    printf("A regiszter tartalma: %d\n",rx_data[1]);
    return(rx_data[1]);
}


/**                    több regiszter olvasása
 *  
 *  Meg kell adni a címet, egy tömböt amelybe az eredményt rögzítjük, illetve a regiszterek számát
 *  1-gyel kell kezdődnie az adatnak, 2-2 bájtot olvasunk ki a megadott regiszterekből sorjában
 */
void MPU9250::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )
{
    unsigned int  i = 0;
    for(i = 0; i < Bytes; i++)
    {
        uint8_t a=0b10000000;
        uint8_t b=0x00;
        a =((ReadAddr+i)|a);
        uint8_t tx_data[2] = { a, b };
        uint8_t rx_data[2] = { 0xFF, 0xFF};
    
    spi_transaction_t t = {
        .length = 2*8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data
     };  
     
     ESP_ERROR_CHECK(spi_device_polling_transmit(spi_dev_mpu9250, &t));
     ReadBuf[i] = rx_data[1];
    }
}


/*                                     INICIALIZÁLÁS, magnetométer életre keltése
 * meg kell hívni magnetometer működéséhez, beállítja a megfelelő paramétereket, 
 * mint például az energiamenedzsment módját, a mérési tartományokat.
 * Lehetséges sávszélesség értékek:
 * 
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_184HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ 
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 */
#define MPU_InitRegNum 17

bool MPU9250::init(bool calib_gyro, bool calib_acc){

    initialize_acc_and_gyro(BITSFS_2G,BITSFS_250,calib_acc,calib_gyro);
    uint8_t i = 0;
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = { //[17][2]
        {BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset
        {0x01, MPUREG_PWR_MGMT_1},               // Órajel forrása
        {0x00, MPUREG_PWR_MGMT_2},               // Acc & Gyro engedélyezve
        {parameters.my_low_pass_filter, MPUREG_CONFIG},     // DLPF használata, a giroszkóp 184Hz-es sávszélességű.
        {BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
        {BITS_FS_2G, MPUREG_ACCEL_CONFIG},       // +-2G
        {parameters.my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Gyorsulásmérőhoz tartozó DLPF, a sávszélessége 184Hz
        {0x12, MPUREG_INT_PIN_CFG},      
        {0x30, MPUREG_USER_CTRL},        // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL},     // I2C 400KHz-es kommunikáció
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //  I2C slave címének beállítása, írás opció kiválasztása 

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 regiszter címe, ahonnan az adatok kiolvasása indul
        {0x01, MPUREG_I2C_SLV0_DO},   // AK8963 újraindítása
        {0x81, MPUREG_I2C_SLV0_CTRL}, // I2C engedélyezése, 1 bájt kiolvasása/írása

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 regiszter címe, ahonnan az adatok kiolvasása indul
#ifdef AK8963FASTMODE
        {0x16, MPUREG_I2C_SLV0_DO},   // 100-Hz es  mérés, jelen esetben nem használt
#else
        {0x12, MPUREG_I2C_SLV0_DO},   // 16bites felbontás beállítása
#endif
        {0x81, MPUREG_I2C_SLV0_CTRL}  // I2C engedélyezése, 1 bájt kiolvasása/írása
        
    };
    for(i = 0; i < MPU_InitRegNum; i++) {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);   
        delay(10); // I2C-nek nem szabad túl gyorsnak lennie
    }
    calib_mag();  
    return 0;
}


/**                                Gyorsulásmérő skálázása
 *
 * 2,4,8 és 16 G értékek beállítására ad lehetőséget FS tartományként a gyorsulásmérőnek
 * visszatérési értéke a ténylegesen beállított skála
 */
unsigned int MPU9250::set_acc_scale(Accelometer_Scale scale){
    int scale1=(int)scale;
    WriteReg(MPUREG_ACCEL_CONFIG, scale1);
    switch(scale1){ //kiolvasásnál a FS-nek megfelelő számmal kell majd elosztani az értéket, ezt itt választjuk ki 
        case BITS_FS_2G: acc_divider=16384; break;
        case BITS_FS_4G: acc_divider=8192;  break;
        case BITS_FS_8G: acc_divider=4096;  break;
        case BITS_FS_16G: acc_divider=2048; break;   
    }
    parameters.a_scale = WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
    switch (parameters.a_scale){ // Beállítjuk a struktúra megfelelő paraméterét
        case BITS_FS_2G:  parameters.a_scale=2;     break;
        case BITS_FS_4G:  parameters.a_scale=4;     break;
        case BITS_FS_8G:  parameters.a_scale=8;    break;
        case BITS_FS_16G: parameters.a_scale=16;    break;   
    }
    return parameters.a_scale;
}



/**                                giroszkóp skálázása
 *
 * 250,500,1000 és 2000 DPS (fok per másodperc) értékek beállítására ad lehetőséget FS tartományként a giroszkópnak
 * visszatérési értéke a ténylegesen beállított skála
 */
unsigned int MPU9250::set_gyro_scale(Gyro_Scale scale){
    
    WriteReg(MPUREG_GYRO_CONFIG, 24);
    switch (scale){  
        //kiolvasásnál a FS-nek megfelelő számmal kell majd elosztani az értéket, ezt itt választjuk ki 
        case BITSFS_250:   gyro_divider = 131;  break;
        case BITSFS_500:   gyro_divider = 65.5; break;
        case BITSFS_1000:  gyro_divider = 32.8; break;
        case BITSFS_2000:  gyro_divider = 16.4; break;   
    }
    parameters.g_scale = WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
    switch (parameters.g_scale){ // Beállítjuk a struktúra megfelelő paraméterét
        case BITS_FS_250DPS:   parameters.g_scale = 250;   break;
        case BITS_FS_500DPS:   parameters.g_scale = 500;   break;
        case BITS_FS_1000DPS:  parameters.g_scale = 1000;  break;
        case BITS_FS_2000DPS:  parameters.g_scale = 2000;  break;   
    }
    return parameters.g_scale;
}



/**                                 WHO AM I?
 * SPI tesztelésére használható, a WHOAMI regiszter  értékét adja vissza
 * Megfelelő esetben ez a szenzor 0x73-mal (115) tér vissza 
 * (Habár adatlap szerint 71-gyel kellene, de ez betudható egy tervezés során történt hibának)
 */
unsigned int MPU9250::whoami(){
    unsigned int response;
    response = ReadReg(MPUREG_WHOAMI, 0x00);
    return response;
}



/*                                 gyorsulásmérő kiolvasása
 * A függvény kigyűjti a gyorsulásmérő adta aktuális adatokat. (3 (x,y,z)*2 (alsó, és felső)=6 bájtnyi adat)
 * Ezt követően a accel_data[0,1,2] változóba x,y,z koordináták szerint rendezi az adatokat.
 */
void MPU9250::read_acc()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
        data = (float)bit_data;
        accel_data[i] = data/acc_divider - a_bias[i];
    }   
}

/*                                 Giroszkóp kiolvasása
 * A függvény kigyűjti a giroszkóp adta aktuális adatokat. (3 (x,y,z)*2 (alsó, és felső) bájt)
 * Ezt követően a gyro_data[0,1,2] változóba x,y,z koordináták szerint rendezi az adatokat.
 */
void MPU9250::read_gyro()
{
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
    for(i = 0; i < 3; i++) {
        bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
        data = (float)bit_data;
        gyro_data[i] = data/gyro_divider- g_bias[i];
    }
}


/*                                 gyorsulásmérő kézi kalibrálása
 * X,Y és Z irányú offset értéke kézzel állítható ezen függvény segítségével.
 */
void MPU9250::calib_acc(float XA, float YA, float ZA)
{
    parameters.a_offset=true;
    a_bias[0] = XA;  
    a_bias[1] = YA;
    a_bias[2] = ZA;
}

/*                                 gyorsulásmérő automatikus kalibrálása
 * A függvény kigyűjti a gyorsulásmérő adta értékeket (50db)-ot.
 *Majd ezeknek az átlagát véve ezt adjuk offset értékeknek.
 *Ennél ez különösen fontos, hiszen Z irányban egy elég nagy alap offset figyelhető meg.
 *Fontos, hogy amíg a függvény ezt a számítást elvégzi, legyen a szenzorunk álló helyzetben.
 */
void MPU9250::auto_calib_acc() 
{
    parameters.a_offset=true;
    int ii;
    float datas[3]={0.0,0.0,0.0};
    for (ii = 0; ii < 50; ii++) {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        read_acc();
        datas[0]+=accel_data[0];
        datas[1]+=accel_data[1];
        datas[2]+=accel_data[2];           
    }
    a_bias[0] = datas[0]/50;  
    a_bias[1] = datas[1]/50; 
    a_bias[2] = datas[2]/50;
}


/*                                 Giroszkóp kézi kalibrálása
 * X,Y és Z irányú offset értéke kézzel állítható ezen függvény segítségével.
 */
void MPU9250::calib_gyro(float XG, float YG, float ZG)
{
    parameters.g_offset=true;
    g_bias[0] = XG;  
    g_bias[1] = YG;
    g_bias[2] = ZG;
}


/*                                 Giroszkóp automatikus kalibrálása
 * A függvény kigyűjti a giroszkóp adta szögelfordulás értékeket (50db)-ot.
 *Majd ezeknek az átlagát véve ezt adjuk offset értékeknek.
 *Fontos, hogy amíg a függvény ezt a számítást elvégzi, legyen a szenzorunk álló helyzetben.
 */
void MPU9250::auto_calib_gyro() 
{
    parameters.g_offset=true;
    int ii;
    float datas[3]={0.0,0.0,0.0};
    for (ii = 0; ii < 50; ii++) {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        read_gyro();
        datas[0]+=gyro_data[0];
        datas[1]+=gyro_data[1];
        datas[2]+=gyro_data[2];           
    }
    g_bias[0] = datas[0]/50;  
    g_bias[1] = datas[1]/50; 
    g_bias[2] = datas[2]/50;
}


/**                                magnetométer skálázása
 *
 * 2 féle skála beállítására ad lehetőséget, a mágnesesség értékeket miliGaussban adjuk meg.
 * visszatérési értéke a ténylegesen beállított skála
 */
float MPU9250::set_mag_scale(Magneto_Scale scale){
  switch (scale)
  {
 
  // 14 és 16 bites skála beállítására ad lehetősége, a Full Scale értéke nem állítható
    case BITSFS_14:
          parameters.mag_scale = 10.*4912./8190.; // megfelelő skálázáshoz, hogy milliGauss-t kapjunk.
        
          break;
    case BITSFS_16:
          parameters.mag_scale = 10.*4912./32760.0;  // megfelelő skálázáshoz, hogy milliGauss-t kapjunk.
          
          break;
  }
  return parameters.mag_scale;
}


/**                                magnetométer kalibrálása
 *
 * A magnetométer érzékenységéhez szükséges változók tudhatóak meg a segítségével, olvasás előtt
 * ezt a függvényt mindenképpen meg kell hívni
 *
 */
void MPU9250::calib_mag(){
    parameters.mag_offset=true;
    uint8_t response[3];
    float data;
    int i;
    // 14 vagy 16 bites felbontás választása
    uint8_t MFS_14BITS = 0; // 0.6 mG/LSB
    //uint8_t MFS_16BITS =1; // 0.15 mG/LSB
    
    uint8_t M_8HZ = 0x02; // 8 Hz-enként olvasson
    //uint8_t M_100HZ = 0x06; // 100 Hz-enként olvasson


    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);   
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);                 
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);                       
    delay(100);
    
    WriteReg(AK8963_CNTL1, 0x00);                    
    delay(50);                                               
    WriteReg(AK8963_CNTL1, 0x0F);                              
    delay(50);                                              

    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    for(i = 0; i < 3; i++) {
        data=response[i];
        Magnetometer_ASA[i] = (((data-128)*0.5)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
    }
    WriteReg(AK8963_CNTL1, 0x00); 
    delay(50);  

    // A magnetométer konfigurálása folyamatos olvasásra, a legjobb felbontást beállítva
    // A 4. bitet 0/1-be állítva 16/14 bites felbontás elérhető
    // 0010-et állítsunk be 8 Hz-es , 0110-t 100 Hz es mintavételért. 
    WriteReg(AK8963_CNTL1, MFS_14BITS << 4 | M_8HZ);   
    delay(50);  
}


/**                                magnetométer olvasása
 *
 * A megnetométer regisztereit akarjuk használni, eezért I2C-n beállítjuk az ehhez szükséges paramétereket.
 * ezt követően a szenzor adataiból X,Y és Z irányban, mindenhol felső és alsó bájtokat összefűzve
 * megkapjuk a megnetométer adott irányú értékét mGaussban.
 */
void MPU9250::read_mag(){
    uint8_t response[7];
    float data;
    int i;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);  // I2C slace címének beállítása, írás módot választva
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                 // Beállítjuk, hogy honnan kezdjen olvasni
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                      // 7 bájt kiolvasására van szükségünk

    const TickType_t delay= 100/portTICK_PERIOD_MS;
    vTaskDelay(delay);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);
    for(i = 0; i < 3; i++) {
        mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
        data = (float)mag_data_raw[i];
        mag_data[i] = data*Magnetometer_ASA[i]-m_bias[i];
    }
}


/**                                magnetométer whoami
 *
 * A megnetométer regisztereit akarjuk használni, ezért I2C-n beállítjuk az ehhez szükséges paramétereket.
 * Ezt követően csak kiolvassuk a kívánt regisztert.
 * visszatérési értéke a WHOAMI regiszter értéke (72 a headerben beállított érték.)
 */
uint8_t MPU9250::AK8963_whoami(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // I2C slave címét beállítjuk, olvasás módban használjuk.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA);  // I2C slave 0 regiszter kezdőcíme, ahonnan a tranzakció kezdődni fog.
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);  //Felső bájt mindig 8-as, alsó, hogy hány darab bájtra vagyunk kíváncsiak.

    const TickType_t delay= 1000/portTICK_PERIOD_MS; //nem a legszebb megoldás, de biztosítani kell, hogy az SPI-on keresztül történő kommunikáció végbemenjen
    vTaskDelay(delay);                  // minden magnetométer regiszteres műveletnél a fentebbi I2C paraméterezést, és ezt a késleltetést is meg kell ejteni.

    response=WriteReg(MPUREG_EXT_SENS_DATA_00,0x00 );    //Egyszerű kiolvasás
    ReadReg(MPUREG_EXT_SENS_DATA_00, 0x00);
    return response;
}


/**                                magnetométer CNTL1 regisztere
 *
 * A megnetométer regisztereit akarjuk használni, ezért I2C-n beállítjuk az ehhez szükséges paramétereket.
 * Ezt követően csak kiolvassuk a kívánt regisztert.
 * visszatérési értéke a CNTL1 regiszter értéke
 */
uint8_t MPU9250::get_CNTL1(){
    uint8_t response;
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG); // I2C slave címét beállítjuk, olvasás módban használjuk.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_CNTL1 );              // I2C slave 0 regiszter kezdőcíme, ahonnan a tranzakció kezdődni fog.
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81); //Felső bájt mindig 8-as, alsó, hogy hány darab bájtra vagyunk kíváncsiak.

    const TickType_t delay= 1000/portTICK_PERIOD_MS;
    vTaskDelay(delay);

    response=WriteReg(MPUREG_EXT_SENS_DATA_00,0x00 );    
    ReadReg(MPUREG_EXT_SENS_DATA_00, 0x00);
        return  response; 
} 
 

/**                                Gyorsulásmérő, giroszkóp és magnetométer értékeinek kiovasása
 *
 * Ahhoz, hogy egy egyértelműen meghatározható pozíciót kinyerjünk, mind a három szenzor adatait kiolvassuk.
 *
 */
void MPU9250::read_all(){
    read_acc();
    read_gyro();
    read_mag();
}


/*                                 magnetométer kézi kalibrálása, kemény mágneses anyag
* Keménymágneses zavarás hatása küszöbölhető a segítségével
 * X,Y és Z irányú offset értéke kézzel megadható ezen függvény segítségével.
 */
void MPU9250::calib_offs_mag(float XM, float YM, float ZM)
{
    parameters.mag_offset=true;
    m_bias[0] = XM;  
    m_bias[1] = YM;
    m_bias[2] = ZM;
}

/*                                 magnetométer kézi kalibrálása
*  Lágymágneses zavarás hatása küszöbölhető a segítségével
 * Kör helyett egy fordulat megtétele után ellipszist kapunk ráadásul nem is az origó lesz a középpontban
 * Ezt az ellipszist teszi origó középpontúvá illetve alakítja körré eza  függvény 
 * Bemeneti paraméterei: A főtengely és a kistengelyre eső két pont x é y koordinátája
 */
void MPU9250::calib_soft_iron_mag(float X1, float Y1,float X2, float Y2)
{
    parameters.mag_offset=true;
    //eltolás, hogy (0,0) legyen a középpont
    float r=sqrt(pow(X1, 2)+pow(Y1, 2));
    float theta=asin(Y1/r);
    float Y2v=  (sin(theta)*X2*-1)+(cos(theta)*Y2);
    //Ezt követően az ellipszisből skálázással kört csinálunk
    float omega=Y2v/r;
    float m_data_0 = (omega*(cos(theta)* mag_data[0]+(sin(theta)* mag_data[1])));
    float m_data_1 =(sin(theta)*mag_data[0]*-1)+(cos(theta)*mag_data[1]);
    mag_data[0]= m_data_0;
    mag_data[1]=m_data_1;
}

/*                                 Magnetométer automatikus kalibrálása
 * A függvény összegyűjt a magnetométer adta értékekből 100db-ot.
 * Majd ezeknek az átlagát véve ezt adjuk offset értékeknek.
 * Legyünk tekintettel arra tényre, hogy míg a gyorsulásmérő és a giroszkóp esetben állóhelyzetben az alap érétkeknek 0-nak kellene lennie, 
 * A magnetométer esetén mindig megfigyelhető lesz a környezet mágneses tere miatt egy bizonyos szintű mágnesesség
 * Ennek kinullázása nem feltétlenül ad helyes eredményt, de mivel nincs más eszközünk a mágnesesség értékek helyességének megmérésére, 
 * ezért ezt a függvényt is nyugodtan alkalmazhatjuk.
 */
void MPU9250::auto_calib_mag() 
{
    parameters.mag_offset=true;
    int ii;
    float datas[3]={0.0,0.0,0.0};
    for (ii = 0; ii < 100; ii++) {
        const TickType_t delay= 50/portTICK_PERIOD_MS;
        vTaskDelay(delay);
        read_mag();
        datas[0]+=mag_data[0];
        datas[1]+=mag_data[1];
        datas[2]+=mag_data[2];           
    }
    m_bias[0] = datas[0]/100;  
    m_bias[1] = datas[1]/100; 
    m_bias[2] = datas[2]/100;
}
