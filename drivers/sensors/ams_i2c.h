/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
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

#ifndef __AMSI2C_H
#define __AMSI2C_H

int ams_i2c_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size);
int ams_i2c_read(struct i2c_client *client, u8 reg, u8 *val);
int ams_i2c_blk_read_direct(struct i2c_client *client,
	u8 reg, u8 *val, int size);
int ams_i2c_write_direct(struct i2c_client *client, u8 reg, u8 val);
int ams_i2c_write(struct i2c_client *client, u8 *shadow, u8 reg, u8 val);
int ams_i2c_reg_blk_write(struct i2c_client *client, u8 reg, u8 *val, int size);
int ams_i2c_ram_blk_write(struct i2c_client *client, u8 reg, u8 *val, int size);
int ams_i2c_ram_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size);
int ams_i2c_modify(struct i2c_client *client,
	u8 *shadow, u8 reg, u8 mask, u8 val);
void ams_i2c_set_field(struct i2c_client *client,
	u8 *shadow, u8 reg, u8 pos, u8 nbits, u8 val);
void ams_i2c_get_field(struct i2c_client *client,
	u8 reg, u8 pos, u8 nbits, u8 *val);

#endif /* __AMSI2C_H */
