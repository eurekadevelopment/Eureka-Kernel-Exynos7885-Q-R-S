#include <linux/string.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <soc/samsung/acpm_mfd.h>

#define CODEC_SPEEDY_ADDR 0x03
#define SUCCESS 0
#define OK 1
#define FAIL -1
#define COMMAND_DUMP 1
#define COMMAND_READ 2
#define COMMAND_WRITE 3
#define MAX_STRING_LENGTH 64
#define ADDR_INDEX_STRING	"    00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\n"

char g_codec_command[MAX_STRING_LENGTH] = "dump";
char reg_dump[1000];
unsigned int g_command, g_arg1, g_arg2;

/*
 * global variables for
 * the codec_speedy_dev_t device number
 * the character device structure
 * the device class
 */
static dev_t codec_speedy_dev_t;
static struct cdev c_dev;
static struct class *class_codec_speedy;

int atoh(char a)
{
	if (a >= '0' && a <= '9')
		return a - '0';

	switch (a) {
	case 'a': return 10;
	case 'b': return 11;
	case 'c': return 12;
	case 'd': return 13;
	case 'e': return 14;
	case 'f': return 15;
	case 'A': return 10;
	case 'B': return 11;
	case 'C': return 12;
	case 'D': return 13;
	case 'E': return 14;
	case 'F': return 15;
	default: return FAIL;
	}
}

int is_available_char(char a)
{
	if (a >= '0' && a <= '9')
		return OK;
	if (a >= 'a' && a <= 'f')
		return OK;
	if (a >= 'A' && a <= 'F')
		return OK;
	if (a == 'x' || a == 'X')
		return OK;

	return FAIL;
}

int str2hex(char *str)
{
	int i = 0, value = 0;
	unsigned int temp_arg1 = 0, temp_arg2 = 0;

	/*
	 * the max strnlen is 4 for the 0x00
	 * but the format could accept like 0, 00, 0x0
	 */
	for (i = 0; i < 5; i++)
		if ((is_available_char(*(str+i)) == FAIL)
				&& (*(str+i) != 'x')
				&& (*(str+i) != 'X'))
			break;

	if (i <= 0 || i > 4) {
		/* return fail with exceptional cases */
		pr_err("sysfs: wrong arguments\n");
		return FAIL;
	} else if (i == 1) {
		/* in case of the one char hex format
		 * like 0, 1, 2 ... e and f
		 */
		value = atoh(str[0]);
		if (value == FAIL)
			return FAIL;

	} else if (i == 2) {
		/* in case of the 2 chars hex format
		 * like 00, 01, 02 ... 0e and 0f
		 */
		temp_arg1 = atoh(str[0]);
		temp_arg2 = atoh(str[1]);

		if ((temp_arg1 == FAIL) || (temp_arg2 == FAIL)) {
			pr_err("sysfs: temp_args = FAIL.\n");
			return FAIL;
		}
		value = temp_arg1 * 16 + temp_arg2;

	} else if (i == 3 && (str[1] == 'x')) {
		value = atoh(str[2]);
	} else if (i == 4 && (str[1] == 'x')) {
		/* in case of the 4 chars hex format
		 * like 0x00, 0x01, 0x02 ... 0x0e and 0x0f
		 */
		temp_arg1 = atoh(str[2]);
		temp_arg2 = atoh(str[3]);

		if ((temp_arg1 == FAIL) || (temp_arg2 == FAIL)) {
			pr_err("sysfs: temp_args = FAIL.\n");
			return FAIL;
		}
		value = temp_arg1 * 16 + temp_arg2;
	} else {
		pr_err("sysfs: out of length\n");
		return FAIL;
	}

	return value;
}

int command_parsing(char *s1)
{
	unsigned int i = 0;
	const char *delimit = " ";
	char arg1[8], arg2[8];

	/* Skip leading delimiters if new string. */
	if (s1 == NULL)
		return FAIL;
	s1 += strspn(s1, delimit);

	memset(arg1, 0, sizeof(arg1));
	memset(arg2, 0, sizeof(arg2));
	memset(g_codec_command, 0, sizeof(g_codec_command));

	/* extract a command*/
	if (!strncmp(s1, "dump", strlen("dump"))) {
		strncpy(g_codec_command, s1, strlen("dump"));
		g_arg1 = 0;
		g_arg2 = 0;
		g_command = COMMAND_DUMP;

		return SUCCESS;
	} else if (!strncmp(s1, "read", strlen("read"))) {
		strncpy(g_codec_command, s1, strlen("read"));
		g_command = COMMAND_READ;
		s1 = s1 + strlen("read");
	} else if (!strncmp(s1, "write", strlen("write"))) {
		strncpy(g_codec_command, s1, strlen("write"));
		g_command = COMMAND_WRITE;
		s1 = s1 + strlen("write");
	} else
		return FAIL;

	/* check if no more string */
	if (s1 == NULL)
		return FAIL;

	/* extracting first arg */
	while (*(s1++) == ' ')
		continue;

	s1--;
	i = 0;

	while (is_available_char(*s1) != FAIL) {
		arg1[i] = *s1;
		i++;
		s1++;

		if (i >= 8)
			return FAIL;
	}
	arg1[i] = '\0';

	g_arg1 = str2hex(arg1);
	if (g_arg1 == FAIL)
		return FAIL;

	/* return after have 1 arg */
	if (g_command == COMMAND_READ)
		return SUCCESS;

	g_command = COMMAND_WRITE;

	while (*s1 == ' ')
		s1++;

	/* extracting second arg */
	i = 0;
	while ((*s1 != ' ') && (*s1 != '\0')) {
		if ((*s1 != 'X')
				&& (*s1 != 'x')
				&& (is_available_char(*s1) == FAIL))
			break;

		arg2[i] = *(s1);
		if (i > 4)
			break;
		i++; s1++;
	}
	g_arg2 = str2hex(arg2);
	if (g_arg2 == FAIL)
		return FAIL;

	return SUCCESS;
}

static int codec_speedy_open(struct inode *i, struct file *f)
{
	pr_info("Driver: open()\n");
	return 0;
}
static int codec_speedy_close(struct inode *i, struct file *f)
{
	pr_info("Driver: close()\n");
	return 0;
}

void value2hex(unsigned int value, char *temp)
{
	unsigned int first, second;

	first = (value & 0x00F0) / 16;
	second = value & 0x000F;
	switch (first) {
	case 0xa:
		*temp = 'a';
		break;
	case 0xb:
		*temp = 'b';
		break;
	case 0xc:
		*temp = 'c';
		break;
	case 0xd:
		*temp = 'd';
		break;
	case 0xe:
		*temp = 'e';
		break;
	case 0xf:
		*temp = 'f';
		break;
	default:
		*temp = first + '0';
		break;
	}

	switch (second) {
	case 0xa:
		*(temp+1) = 'a';
		break;
	case 0xb:
		*(temp+1) = 'b';
		break;
	case 0xc:
		*(temp+1) = 'c';
		break;
	case 0xd:
		*(temp+1) = 'd';
		break;
	case 0xe:
		*(temp+1) = 'e';
		break;
	case 0xf:
		*(temp+1) = 'f';
		break;
	default:
		*(temp+1) = second + '0';
		break;
	}

	*(temp+2) = '\0';
}

static ssize_t codec_speedy_read(struct file *f,
		char __user *buf,
		size_t len,
		loff_t *off)
{
	unsigned int value, i;
	char temp[3];

	pr_info("sysfs driver: read()\n");

	memset(reg_dump, 0, sizeof(reg_dump));
	if (g_command == COMMAND_DUMP) {

		strncpy(reg_dump,
				ADDR_INDEX_STRING,
				sizeof(ADDR_INDEX_STRING));

		/* register addresses 0x00 ~ 0xFF(255) */
		for (i = 0; i <= 255; i++) {
			/* line numbers like 0x10, 0x20, 0x30 ... 0xff
			 * in the first colume
			 */
			if (i % 16 == 0) {
				value2hex(i, temp);
				strcat(reg_dump, temp);
				strcat(reg_dump, ":");
			}

			exynos_acpm_read_reg(CODEC_SPEEDY_ADDR,
					i,
					(u8 *)&value);

			strcat(reg_dump, " ");
			value2hex(value, temp);
			strcat(reg_dump, temp);

			if (i % 16 == 15)
				strcat(reg_dump, "\n");
		}
	} else if (g_command == COMMAND_READ) {
		exynos_acpm_read_reg(CODEC_SPEEDY_ADDR, g_arg1, (u8 *)&value);
		g_arg2 = (unsigned int)value & 0x00ff;
	}

	return 0;
}
static ssize_t codec_speedy_write(struct file *f,
		const char __user *buf,
		size_t len,
		loff_t *off)
{
	pr_info("sysfs driver: write()\n");
	exynos_acpm_write_reg(CODEC_SPEEDY_ADDR, g_arg1, g_arg2);
	return len;
}

static const struct file_operations pugs_fops = {
	.owner = THIS_MODULE,
	.open = codec_speedy_open,
	.release = codec_speedy_close,
	.read = codec_speedy_read,
	.write = codec_speedy_write
};

static ssize_t sysfs_show_func(struct class *cls,
		struct class_attribute *attr,
		char *buf)
{
	codec_speedy_read(0, 0, 0, 0);

	if (g_command == COMMAND_DUMP)
		return sprintf(buf, "%s", reg_dump);
	if (g_command == COMMAND_READ)
		return sprintf(buf, "0x%x\n", g_arg2);
	if (g_command == COMMAND_WRITE)
		return sprintf(buf, "0x%x\n", g_arg2);

	return sprintf(buf, "%s", reg_dump);
}

static ssize_t sysfs_store_func(struct class *cls,
		struct class_attribute *attr,
		const char *buf,
		size_t count)
{
	int ret;
	char input[MAX_STRING_LENGTH];

	if (strlen(buf) > MAX_STRING_LENGTH) {
		pr_info("%s() [ERROR] wrong input: %s\n", __func__, buf);
		return 0;
	}

	memcpy(input, buf, strlen(buf));
	ret = command_parsing(input);

	if (g_command == COMMAND_WRITE)
		codec_speedy_write(0, 0, 0, 0);

	return count;
}


static CLASS_ATTR(codec_sysfs,
		0664,
		sysfs_show_func, sysfs_store_func);

static int __init codec_speedy_init(void) /* Constructor */
{
	int ret = 0;

	pr_info("codec_speedy: registered");
	if (alloc_chrdev_region(&codec_speedy_dev_t, 0, 1, "acpm") < 0)
		return -1;

	class_codec_speedy = class_create(THIS_MODULE, "codec_speedy");
	if (class_codec_speedy == NULL) {
		unregister_chrdev_region(codec_speedy_dev_t, 1);
		return -1;
	}
	if (device_create(class_codec_speedy,
				NULL,
				codec_speedy_dev_t,
				NULL, "acpm") == NULL) {
		class_destroy(class_codec_speedy);
		unregister_chrdev_region(codec_speedy_dev_t, 1);
		return -1;
	}
	cdev_init(&c_dev, &pugs_fops);
	if (cdev_add(&c_dev, codec_speedy_dev_t, 1) == FAIL) {
		device_destroy(class_codec_speedy, codec_speedy_dev_t);
		class_destroy(class_codec_speedy);
		unregister_chrdev_region(codec_speedy_dev_t, 1);
		return -1;
	}

	ret = class_create_file(class_codec_speedy,
			&class_attr_codec_sysfs);
	if (ret)
		pr_info("%s() [Error] class_create_file\n", __func__);

	g_command = COMMAND_DUMP;

	return 0;
}

static void __exit codec_speedy_exit(void) /* Destructor */
{
	cdev_del(&c_dev);
	device_destroy(class_codec_speedy, codec_speedy_dev_t);
	class_destroy(class_codec_speedy);
	unregister_chrdev_region(codec_speedy_dev_t, 1);
	pr_info("codec_speedy: unregistered");
}

module_init(codec_speedy_init);
module_exit(codec_speedy_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jason Seong<jason.seong@samsung.com>");
MODULE_DESCRIPTION("Codec Speedy");
