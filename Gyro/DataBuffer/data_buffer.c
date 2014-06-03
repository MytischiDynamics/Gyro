#include "data_buffer.h"

gyro_error InitDataBuffer(data_buffer* db, uint16_t block_size, int16_t* buffer)
{
	gyro_error err = NO_ERROR;
	if (db == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}
	if (block_size == 0) {
		err = ERROR_VALUE_NOT_IN_RANGE;
		goto err_occured;
	}

	db->counter = 0;
	db->size = block_size;
	db->buffer = buffer;
	db->current_session_buffer_start = db->buffer;

err_occured :
	return err;
}

gyro_error WriteValue(data_buffer *db, int16_t value)
{
	gyro_error err = NO_ERROR;
	if (db == NULL) {
		err = ERROR_NULL_POINTER;
		goto err_occured;
	}

	if (db->counter > db->size - 1) {
		db->counter = 0;
		if (db->current_session_buffer_start == db->buffer + db->size) {
			db->current_session_buffer_start = db->buffer;
		} else if (db->current_session_buffer_start == db->buffer) {
			db->current_session_buffer_start = db->buffer + db->size;
		}
	}
	*(db->current_session_buffer_start + db->counter) = value;
	db->counter++;

err_occured:
	return err;

}