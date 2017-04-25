/* mbed Microcontroller Library
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "DataFlashDevice.h"

#define SPIF_TIMEOUT    10000


enum ops {
    DF_ID  = 0x9F, // Read ID
    DF_STATUS = 0xD7, // Read Status
    DF_CAR_HF_2 = 0x1B, // Continuous Array Read (High Frequency)
    DF_BUF_1_WRITE   = 0x84, // Buffer 1 Write 
    DF_BUF_1_PROG_W_ERASE   = 0x83, // Buffer 1 to Main Memory Page Program with Built-In Erase
	DF_BUF_1_WRITE_W_PROG_W_ERASE   = 0x82, // Main Memory Page Program through Buffer 1 with Built-In Erase
    DF_PAGE_ERASE = 0x81, // Page Erase 

};

enum density {
    DF_2MB	= 0x03, // 2Mbits
    DF_4MB	= 0x04, // 4Mbits
    DF_8MB	= 0x05, // 8Mbits
    DF_16MB = 0x06, // 16Mbits 
    DF_32MB = 0x07, // 32Mbits 
    DF_64MB = 0x08, // 64Mbits
};

DataFlashDevice::DataFlashDevice(PinName mosi, PinName miso, PinName sclk,
		PinName cs, PinName wp, int freq) :
		_spi(mosi, miso, sclk), _cs(cs), _wp(wp), _size(0) {
	_cs = 1;
	_wp = 0;
	_spi.frequency(freq);
}
int DataFlashDevice::id() {
	int id = 0;
	_cs = 0;
	_spi.write(DF_ID);
	id = (_spi.write(0x00) << 8);
	id |= _spi.write(0x00);
	_cs = 1;
	return id;
}

void DataFlashDevice::wren(bool en) {
	_wp = en;

}
// return the Status
int DataFlashDevice::status() {
	int status = 0;
	_cs = 0;
	_spi.write(DF_STATUS);
	status = (_spi.write(0x00));
	_cs = 1;
	return status;
}

int DataFlashDevice::init() {
	int _id = 0;
	int _status = 0;
	int density =0;
	bool binary = 0;
	wren(1);
	_id = id();
	density = _id & 0x1f;
	_status = status();
	binary = _status & 0x1;

	if (density == DF_2MB)  // 2Mbits
			{

		pages = 1024;        // Number of pages
		blocks = 128;        // Number of blocks
		if (binary) {
			pagesize = 256;
		} else {
			pagesize = 264;
		}
		devicesize = pages * pagesize;

	} else if (density == DF_4MB) // 4Mbits
			{

		pages = 2048;
		blocks = 256;
		if (binary) {
			pagesize = 256;
		} else {
			pagesize = 264;
		}
		devicesize = pages * pagesize;

	} else if (density == DF_8MB) // 8Mbits
			{

		pages = 4096;
		blocks = 512;
		if (binary) {
			pagesize = 256;
		} else {
			pagesize = 264;
		}
		devicesize = pages * pagesize;

	} else if (density == DF_16MB) // 16Mbits
			{

		pages = 4096;
		blocks = 512;
		if (binary) {
			pagesize = 512;
		} else {
			pagesize = 528;
		}
		devicesize = pages * pagesize;

	} else if (density == DF_32MB) // 32Mbits
			{

		pages = 8192;
		blocks = 1024;
		if (binary) {
			pagesize = 512;
		} else {
			pagesize = 528;
		}
		devicesize = pages * pagesize;

	} else if (density == DF_64MB) // 64Mbits
			{

		pages = 8192;
		blocks = 1024;
		if (_status & 0x1) {
			pagesize = 1024;
		} else {
			pagesize = 1056;
		}
		devicesize = pages * pagesize;
	} else {
		devicesize = -1;
		pages = -1;
		pagesize = -1;
		blocks = -1;
	}
	return 0;
}

int DataFlashDevice::deinit() {
	// Latch write disable just to keep noise
	// from changing the device
	wren(false);
	return 0;
}
void DataFlashDevice::busy() {
	while (isbusy()); 
}
bool DataFlashDevice::isbusy() {
	return (!(status() & 0x80)); 
}
int DataFlashDevice::read(void *buffer, bd_addr_t addr, bd_size_t len) {
	// Check the address and size fit onto the chip.
	MBED_ASSERT(is_valid_read(addr, len));
	int address;

	address = _getpaddr(addr) | _getbaddr(addr);

	busy();

	_cs = 0;

	_spi.write(DF_CAR_HF_2); 
	_sendaddr(address);

	// 2 dummy bytes
	_spi.write(0x00);
	_spi.write(0x00);

	// clock out the data
	char *buf;
	buf = (char*) buffer;
	while (len > 0) {
		*buf = _spi.write(0x00);
		buf++;
		len--;
	}
	_cs = 1;

	busy();
	return 0;
}

int DataFlashDevice::program(const void *buffer, bd_addr_t addr,
		bd_size_t size) {
			//TODO we could check if an enture page can be erased and written saving a bit of time.
	int err;
	// Check the address and size fit onto the chip.
	MBED_ASSERT(is_valid_program(addr, size));
		char *buf;
		buf = (char*) buffer;
	while (size > 0) {
		wren(1);

		_cs = 0;
		
		_spi.write(DF_BUF_1_WRITE); // write to buffer 1
		_sendaddr(0); // write from addr 0

		for (int i = 0; i < pagesize; i++) {
			_spi.write(*buf);
			buf++;
		}
		_cs = 1;

		busy();

		// write buffer 1 to memory with erase.
		_cs = 0;
		_spi.write(DF_BUF_1_PROG_W_ERASE);
		_sendaddr(_getpaddr(addr));
		
		
		_cs = 1;
		
		
		/*
		_cs = 0;
		
		_spi.write(DF_BUF_1_WRITE_W_PROG_W_ERASE); // Main Memory Page Program through Buffer 1 with Built-In Erase
		_sendaddr(_getpaddr(addr)); // write from start of page

		for (int i = 0; i < pagesize; i++) {
			_spi.write(*buf);
			buf++;
		}
		_cs = 1;
*/

		busy();

		addr += pagesize;
		size -= pagesize;

		wait_ms(1);

		err = _sync();
		wren(0);
		if (err) {
			return err;
		}
		
	}

	return 0;
}


int DataFlashDevice::_getpaddr(int address) {

	int paddr;

	if (pagesize == 256) {
		paddr = address & 0xffffff00;
	} else if (pagesize == 264) {
		paddr = (address/264)<<9;
	} else if (pagesize == 512) {
		paddr = address & 0xfffffe00;
	} else if (pagesize == 528) {
		paddr = (address/528)<<10;
	} else if (pagesize == 1024) {
		paddr = address & 0xfffffc00;
	} else if (pagesize == 1056) {
		paddr = (address/1056)<<10;
	} else {
		paddr = -1;
	}

	return (paddr);
}


int DataFlashDevice::_getbaddr(int address) {

	int baddr;

	if (pagesize == 256) {
		baddr = address & 0xff;
	}else if (pagesize == 264) {
		baddr = address %264;
	} else if (pagesize == 512) {
		baddr = address & 0x1ff;
	}else if (pagesize == 528) {
		baddr = address %528;
	} else if (pagesize == 1024) {
		baddr = address & 0x3ff;
	} else if (pagesize == 1056) {
		baddr = address %1056;
	} else {
		baddr = -1;
	}

	return (baddr);
}
int DataFlashDevice::_sync() {
	for (int i = 0; i < SPIF_TIMEOUT; i++) {
		
		 if (!isbusy()) {
		 return 0;
		 }

		 wait_ms(1);
		 
		return 0;

	}

	return BD_ERROR_DEVICE_ERROR;
}
void DataFlashDevice::_sendaddr(int address) {
	_spi.write(address >> 16);
	_spi.write(address >> 8);
	_spi.write(address);
}
int DataFlashDevice::erase(bd_addr_t addr, bd_size_t size) {
//we are using Program with Built-In Erase so no need to erase before program.
	eraseBlock(addr,size);
}

int DataFlashDevice::eraseBlock(bd_addr_t addr, bd_size_t size) {
	// Check the address and size fit onto the chip.
	int err;
	MBED_ASSERT(is_valid_erase(addr, size));

	while (size > 0) {
		wren(1);

		// Erase sector
		// TODO if an entire block or sector can be erases we should do that instead.
		busy();
		_cs = 0;
		_spi.write(DF_PAGE_ERASE);

		_sendaddr(_getpaddr(addr));

		_cs = 1;
		busy();
		addr += pagesize;
		size -= pagesize;
		wren(0);
		err = _sync();
		if (err) {
			return err;
		}
	}

	return 0;
}
//technically smaller reads and non erase writes can be done but not a lot of point.
bd_size_t DataFlashDevice::get_read_size() const {
	return pagesize;
}

bd_size_t DataFlashDevice::get_program_size() const {
	return pagesize;
}

bd_size_t DataFlashDevice::get_erase_size() const {
	return pagesize;
}

bd_size_t DataFlashDevice::size() const {
	return devicesize;
}
