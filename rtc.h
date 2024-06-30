
typedef struct rtc_t rtc_t;

//Memory range access handlers
void rtc_write8(void *obj, unsigned int a, unsigned int val);
void rtc_write16(void *obj, unsigned int a, unsigned int val);
unsigned int rtc_read8(void *obj, unsigned int a);
unsigned int rtc_read16(void *obj, unsigned int a);


rtc_t *rtc_new();

//Call this periodically to make the RTC tick.
void rtc_tick(rtc_t *r, int ticklen_us);
