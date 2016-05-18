typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	int8_t t;
}BMA250E_ACC_TMP;	

/* temperature measure need 3ms */
#define TEMPERATURE_PERIOD                  3
#define TEMPERATURE_PERIODIC_EVT      		0x4000						
/* pressure measure need 10ms */
#define PRESSURE_PERIOD    	                10
#define PRESSURE_PERIODIC_EVT    			0x8000
