#include <assert.h>
#include <signal.h>

extern uint32_t fletcher32( uint16_t const *data, size_t words );
extern uint16_t fletcher16( uint8_t const *data, size_t bytes );


static inline void __CHECK(const char *func, const char *name, void *data, uint32_t count) {
	uint32_t	sum=0;

	printf("%s; %s @ 0x%08x", __FUNCTION__, name, (uint32_t)data);
	if(NULL == data) {
		printf("\n");
		return;
 	}

	if(count & 0x01) {
		sum = fletcher32((uint16_t const *)data, count);
	} else {
		sum = fletcher16((uint8_t const *)data, count);
	}

	printf(" -> 0x%08x\n", sum);
}

#define _CHECK(data, count) __CHECK(__FUNCTION__, #data, data, count)

#if 1
#define CHECK()
#else
#define CHECK() { \
	_CHECK(avr, sizeof(*avr)); \
	_CHECK(&avr->gdb_padding, sizeof(avr->gdb_padding)); \
	_CHECK(&avr->gdb_port_padding, sizeof(avr->gdb_port_padding)); \
	_CHECK(&avr->vcd_padding, sizeof(avr->vcd_padding)); \
 }

//	_CHECK(avr->uflash, sizeof(*avr->uflash)); \

#endif
