#ifndef CFG_EEPROM_H__
#define CFG_EEPROM_H__

#include <EEPROM.h>

class CFG_EEPROM {
public:

	CFG_EEPROM(uint32_t prog_sign)
		: prog_sign_(prog_sign), data_addr_() {

	}

	void Setup() {

		N(); S("SETUP CFG("); SH(prog_sign_); S(")");

		for (auto it = EEPROM.begin(); it != EEPROM.end(); ++it) {

			data_addr_ = it.index;
			N(); S(" SIGN CFG["); SH(data_addr_); S("]: ");

			uint32_t data = EEPROM.get(data_addr_, data); SH(data);
			if (data == 0xffffffff) return;
			if (data == prog_sign_) {

				S(" PROGRAM SIGNATURE MATCH");

				Load();
				return;
			}
		}
	}

	virtual void Load() =0;
	virtual void Save() =0;

protected:

	uint16_t LoadData(void* buffer, uint16_t buffer_size) {

		uint16_t
			load_size = 0, data_size = 0,
			size_addr = data_addr_ + sizeof(prog_sign_),
			load_addr = size_addr + sizeof(data_size),
			load_stop = load_addr + EEPROM.get(size_addr, data_size);

		N(); S(" LOAD CFG["); S(size_addr); S("]: "); S(data_size); S(" BYTES OF DATA ");

		for (uint8_t *data = (uint8_t*)buffer, *data_stop = data + buffer_size;
			data < data_stop && load_addr < load_stop;
			++data, ++load_addr, --load_size
			)
			SH(EEPROM.get(load_addr, *data));

		S(" LOADED "); S(load_size); S(" BYTES");

		return  load_size;
	}

	uint16_t SaveData(void* data, uint16_t data_size) {

		N(); S(" SAVE CFG["); S(data_addr_); S("]: "); SH(prog_sign_); S(" PROGRAM SIGNATURE");

		EEPROM.put(data_addr_, prog_sign_);

		uint16_t
			save_size = 0,
			size_addr = data_addr_ + sizeof(prog_sign_),
			save_addr = size_addr + sizeof(data_size),
			save_stop = EEPROM.end();

		N(); S(" SAVE CFG["); S(size_addr); S("]: "); S(data_size); S(" BYTES OF DATA ");

		for (uint8_t *save_data = (uint8_t*)data, *data_stop = save_data + data_size;
			save_data < data_stop && save_addr != save_stop;
			++save_data, ++save_addr, ++save_size
			)
			SH(EEPROM.put(save_addr, *save_data));

		EEPROM.put(size_addr, save_size);
		S(" SAVED "); S(save_size); S(" BYTES");

		return save_size;
	}

private:

	uint32_t prog_sign_; // program signature, identifies program/version that owns data
	uint16_t data_addr_; // data address at EEPROM
};

class CFG : public CFG_EEPROM {
public:

	CFG(uint32_t prog_sign) :
		CFG_EEPROM(prog_sign), items_(), count_() {

	}

	void Load() {

		count_ = LoadData(items_, sizeof(items_)) / sizeof(items_[0]);
	}

	void Save() {

		SaveData(items_, count_ * sizeof(items_[0]));
	}

	bool Exists(uint32_t value) {

		for(uint32_t *item = items_, *end = item + count_;
			item < end;
			++item
			)
			if (*item == value)
				return true;

		return false;
	}

	bool Add(uint32_t value) {

		if (count_ + 1 > sizeof(items_) / sizeof(items_[0]))
			return false;

		items_[count_++] = value;

		S(" ADD CFG("); SH(value); S(") ");
		return true;

	}

	bool Remove(uint32_t value) {

		bool result = false;

		for(uint32_t *item = items_, *end = item + count_;
			item < end;
			++item
			)
			if (result)
				*(item - 1) = *item;
			else
				if (*item == value) {
					count_--;
					result = true;
					S(" REMOVE CFG("); SH(value); S(") ");
				}

		return result;
	}

private:

	uint32_t items_[128];
	uint16_t count_; 
};

#endif // CFG_EEPROM_H__
