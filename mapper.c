#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "csr.h"
#include "emu.h"
#include "log.h"
#include "mapper.h"

// Debug logging
#define MAPPER_LOG(msg_level, format_and_args...) \
	log_printf(LOG_SRC_MAPPER, msg_level, format_and_args)
#define MAPPER_LOG_DEBUG(format_and_args...) MAPPER_LOG(LOG_DEBUG, format_and_args)
#define MAPPER_LOG_INFO(format_and_args...) MAPPER_LOG(LOG_INFO, format_and_args)

/*
On the MMU:

We have 2048 entries (well, x2, one set for sys and one for usr)
This is virtual memory; if the address space is 8M, this means
a page size of 4K.

We have space for 8K physical pages; this means we could map
to 32MiB of physical RAM.

*/

//Note: on write of 32-bit, first word is msb and second word is lsb
typedef struct {
	uint16_t upper_word;
	uint16_t lower_word;
} desc_t;

#define NUM_PAGE_ENTRIES 4096
#define SYS_ENTRY_START  (NUM_PAGE_ENTRIES/2)

//Note the RWX bits *disable* that access when 1.
#define LOWER_WORD_R 0x8000
#define LOWER_WORD_W 0x4000
#define LOWER_WORD_X 0x2000
#define LOWER_WORD_PAGE_MASK 0x1FFF

#define UPPER_WORD_REFD 0x2
#define UPPER_WORD_ALTRD 0x1
#define UPPER_WORD_UID_SHIFT 8
#define UPPER_WORD_UID_MASK 0xff

struct mapper_t {
	//2K entries for usr, 2K for sys
	desc_t desc[NUM_PAGE_ENTRIES];
	uint8_t *physram;
	int physram_size;
	int sysmode; //indicates if next accesses are in sysmode or not
	int cur_id;
};

void mapper_set_mapid(mapper_t *m, uint8_t id) {
	if (m->cur_id!=id) MAPPER_LOG_DEBUG("Switching to map id %d\n", id);
	m->cur_id=id;
}

//returns fault indicator, or 0 if allowed
static int access_allowed_page(mapper_t *m, unsigned int page, int access_flags) {
	assert(page<NUM_PAGE_ENTRIES);
	unsigned int ac=(m->desc[page].lower_word<<16)+m->desc[page].upper_word;
	int fault=(ac&access_flags)&(ACCESS_R|ACCESS_W|ACCESS_X);
	int uid=(ac>>UPPER_WORD_UID_SHIFT)&UPPER_WORD_UID_MASK;
	if ((access_flags&ACCESS_SYSTEM)==0) {
		if (uid != m->cur_id) fault=(uid<<8|0xff);
	}
	if (fault) {
		MAPPER_LOG_DEBUG("Mapper: Access fault: page ent %x req %x, fault %x (", ac, access_flags, fault);
		if (fault&(LOWER_WORD_W<<16)) MAPPER_LOG_DEBUG("write violation ");
		if (fault&(LOWER_WORD_R<<16)) MAPPER_LOG_DEBUG("read violation ");
		if (fault&(LOWER_WORD_X<<16)) MAPPER_LOG_DEBUG("execute violation ");
		if (fault&0xff00) MAPPER_LOG_DEBUG("proc uid %d page uid %d ", m->cur_id, uid);
		MAPPER_LOG_DEBUG(")\n");
	}
	return fault;
}

int mapper_access_allowed(mapper_t *m, unsigned int a, int access_flags) {
	if (a>=0x800000) {
		//Anything except RAM does not go through the mapper, but is only
		//accessible in system mode.
		int ret=(access_flags&ACCESS_SYSTEM)?ACCESS_ERROR_OK:ACCESS_ERROR_A;
		if (ret==ACCESS_ERROR_A) {
			MAPPER_LOG_INFO("mapper_access_allowed: address %x not accessible in user mode\n", a);
		}
		return ret;
	}
	//Map virtual page to phyical page.
	int p=a>>12; //4K pages
	assert(p<=SYS_ENTRY_START && "out of range addr");
	if (access_flags&ACCESS_SYSTEM) p+=SYS_ENTRY_START;
	int r=access_allowed_page(m, p, access_flags);
	if (r && log_level_active(LOG_SRC_MAPPER, LOG_DEBUG)) {
		MAPPER_LOG_DEBUG("Mapper: Access fault at addr %x page %d. CPU state:\n", a, p);
		dump_cpu_state();
		dump_callstack();
		MAPPER_LOG_DEBUG("Mapper: Dump done.\n");
	}
	int x=ACCESS_ERROR_OK;
	if (r) x=ACCESS_ERROR_A;
	if (r&0xff00) x=ACCESS_ERROR_U;
	return x;
}


void mapper_write16(void *obj, unsigned int a, unsigned int val) {
	if (emu_get_cur_cpu()==0) return; //seems writes from dma cpu are not allowed

	mapper_t *m=(mapper_t*)obj;
	a=a/2; //word addr
	if (a&1) {
		m->desc[a/2].lower_word=val;
	} else {
		m->desc[a/2].upper_word=val;
	}
	// XXX: too many divide by 2?
	if (a/2==2048) MAPPER_LOG_DEBUG("write page %d, w%d. upper_word=%x, lower_word=%x\n", a/2, a&1, m->desc[a/2].upper_word, m->desc[a/2].lower_word);
}

void mapper_write32(void *obj, unsigned int a, unsigned int val) {
	mapper_write16(obj, a, val>>16);
	mapper_write16(obj, a+2, val&0xffff);
}


unsigned int mapper_read16(void *obj, unsigned int a) {
	mapper_t *m=(mapper_t*)obj;
	a=a/2; //word addr
	// XXX: too many divide by 2?
	if (a/2==2048) MAPPER_LOG_DEBUG("read page %d, w%d. upper_word=%x, lower_word=%x\n", a/2, a&1, m->desc[a/2].upper_word, m->desc[a/2].lower_word);
	if (a&1) {
		return m->desc[a/2].lower_word;
	} else {
		return m->desc[a/2].upper_word;
	}
}

void mapper_write8(void *obj, unsigned int a, unsigned int val) {
	int v=mapper_read16(obj, a&~1);
	if (a&1) {
		v=(v&0xFF00)|val;
	} else {
		v=(v&0xFF)|(val<<8);
	}
	mapper_write16(obj, a&~1, v);
}

unsigned int mapper_read8(void *obj, unsigned int a) {
	int v=mapper_read16(obj, a&~1);
	if (a&1) {
		return v&0xff;
	} else {
		return v>>8;
	}
}

unsigned int mapper_read32(void *obj, unsigned int a) {
	return (mapper_read16(obj,a)<<16)+mapper_read16(obj, a+2);
}

void mapper_set_sysmode(mapper_t *m, int cpu_in_sysmode) {
	m->sysmode=cpu_in_sysmode;
}

int do_map(mapper_t *m, unsigned int a, unsigned int is_write) {
	//Map virtual page to phyical page.
	int p=a>>12; //4K pages
	assert(p<SYS_ENTRY_START);
	if (m->sysmode) p+=SYS_ENTRY_START;

	m->desc[p].upper_word|=UPPER_WORD_REFD;
	if (is_write) m->desc[p].upper_word|=UPPER_WORD_ALTRD;

	int phys_p=m->desc[p].lower_word&LOWER_WORD_PAGE_MASK;
	int phys=(a&0xFFF)|(phys_p<<12);
	phys&=((8*1024*1024)-1);
//	assert(phys<8*1024*1024);
//	MAPPER_LOG_DEBUG("do_map %s 0x%x to 0x%x, virt page %d phys page %d\n", m->sysmode?"sys":"usr", a, phys, p, phys_p);
	return phys;
}

void mapper_ram_write8(void *obj, unsigned int a, unsigned int val) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 1);
	if (a<0) return;
	buffer[a]=val;
}

void mapper_ram_write16(void *obj, unsigned int a, unsigned int val) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 1);
	if (a<0) return;
	buffer[a]=(val>>8);
	buffer[a+1]=val;
}

void mapper_ram_write32(void *obj, unsigned int a, unsigned int val) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 1);
	if (a<0) return;
	buffer[a]=(val>>24);
	buffer[a+1]=(val>>16);
	buffer[a+2]=(val>>8);
	buffer[a+3]=val;
}

unsigned int mapper_ram_read8(void *obj, unsigned int a) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 0);
	if (a<0) return 0;
	return buffer[a];
}

unsigned int mapper_ram_read16(void *obj, unsigned int a) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 0);
	if (a<0) return 0;
	return buffer[a+1]+(buffer[a]<<8);
}

unsigned int mapper_ram_read32(void *obj, unsigned int a) {
	mapper_t *m=(mapper_t*)obj;
	uint8_t *buffer=m->physram;
	a=do_map(m, a, 0);
	if (a<0) return 0;
	return buffer[a+3]+(buffer[a+2]<<8)+(buffer[a+1]<<16)+(buffer[a]<<24);
}

mapper_t *mapper_new(void *physram, int size) {
	mapper_t *ret=calloc(sizeof(mapper_t), 1);
	ret->physram=(uint8_t*)physram;
	ret->physram_size=size;
	return ret;
}


