/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING	    *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sensor/sensors_core.h>

int ams_i2c_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		SENSOR_ERR("failed at address %x (%d bytes) err = %d\n",
			reg, size, ret);

	return ret;
}

int ams_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	return ams_i2c_blk_read(client, reg, val, 1);
}

int ams_i2c_blk_read_direct(struct i2c_client *client,
	u8 reg, u8 *val, int size)
{
	s32 ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		SENSOR_ERR("failed at address %x (%d bytes) err = %d\n",
			reg, size, ret);

	return ret;
}

int ams_i2c_write_direct(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0)
			SENSOR_ERR("failed to write register %x err = %d\n",
				reg, ret);
	}

	return ret;
}

int ams_i2c_write(struct i2c_client *client, u8 *shadow, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0)
			SENSOR_ERR("failed to write register %x err = %d\n",
				reg, ret);
	}

	shadow[reg] = val;

	return ret;
}

int ams_i2c_reg_blk_write(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		SENSOR_ERR("failed 2X at address %x (%d bytes) err = %d\n",
			reg, size, ret);

	return ret;
}

int ams_i2c_ram_blk_write(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret = 0;
	int i;
	int bsize = 0;
	int breg = reg;
	int validx = 0;
	int maxblocksize = 32;

	for (i = 0; i < size; i += maxblocksize) {
		if ((size - i) >= maxblocksize)
			bsize = maxblocksize;
		else
			bsize = size - i;

		ret = i2c_smbus_write_i2c_block_data(client,
			breg, bsize, &val[validx]);
		if (ret < 0)
			SENSOR_ERR("failed at address %x (%d bytes) err = %d\n",
				reg, size, ret);

		breg += bsize;
		validx += bsize;
	}

	return ret;
}

int ams_i2c_ram_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret = 0;
	int i;
	int bsize = 0;
	int breg = reg;
	int validx = 0;
	int maxblocksize = 32;

	for (i = 0; i < size; i += maxblocksize) {
		if ((size - i) >= maxblocksize)
			bsize = maxblocksize;
		else
			bsize = size - i;

		ret = i2c_smbus_read_i2c_block_data(client, breg,
				bsize, &val[validx]);
		if (ret < 0)
			ret = i2c_smbus_read_i2c_block_data(client,
				breg, bsize, &val[validx]);
		if (ret < 0)
			SENSOR_ERR("failed 2X at address %x (%d bytes) err = %d\n",
				reg, size, ret);

		ret = i2c_smbus_write_i2c_block_data(client, breg,
			bsize, &val[validx]);
		if (ret < 0)
			SENSOR_ERR("failed at address %x (%d bytes) err = %d\n",
				reg, size, ret);

		breg += bsize;
		validx += bsize;
	}

	return ret;
}

int ams_i2c_modify(struct i2c_client *client, u8 *shadow,
	u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 temp;

	ret = ams_i2c_read(client, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret = ams_i2c_write(client, shadow, reg, temp);

	shadow[reg] = temp;

	return ret;
}


void ams_i2c_set_field(struct i2c_client *client, u8 *shadow, u8 reg, u8 pos,
	u8 nbits, u8 val)
{
	u8 tmp;
	u8 mask = (1 << nbits) - 1;

	ams_i2c_read(client, reg, &tmp);
	tmp &= ~(mask << pos);
	tmp |= (val << pos);
	ams_i2c_write(client, shadow, reg, tmp);
}


void ams_i2c_get_field(struct i2c_client *client, u8 reg, u8 pos, u8 nbits,
	u8 *val)
{
	u8 tmp;
	u8 mask = (1 << nbits) - 1;

	ams_i2c_read(client, reg, &tmp);
	tmp &= mask << pos;
	*val = tmp >> pos;
}
