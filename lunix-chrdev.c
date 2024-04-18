/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Vasileios-Panagiotis Miligas >
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
/*
 * Declare a prototype so we can define the "unused" attribute and keep
 * the compiler happy. This function is not yet used, because this helpcode
 * is a stub.
 */
static int __attribute__((unused)) lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *);
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
		
	WARN_ON (!(sensor = state->sensor));
	
	/*Check if the timestamp of the last sensor update is greater
	 *than the timestamp of the last state buffer update
	 */
	return (sensor->msr_data[state->type]->last_update != state->buf_timestamp) ? 1 : 0;
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint16_t data;
	uint32_t timestamp;
	long value;

	if(!lunix_chrdev_state_needs_refresh(state)) 
		return -EAGAIN;

	sensor = state->sensor;
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	
	/* The data of the sensors are updated
	 * on interrupt context so they are protected 
	 * by a spinlock
	 */ 	
	spin_lock(&sensor->lock);
	/* Why use spinlocks? See LDD0 */ 

	/*
	 * Any new data available?
	 */
	
	
	data = sensor->msr_data[state->type]->values[0];
	timestamp = sensor->msr_data[state->type]->last_update;
	spin_unlock(&state->sensor->lock);	
	
	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	
	if(state->type == BATT) {
		value = lookup_voltage[data];
	}
	else if(state->type == TEMP) {
		value = lookup_temperature[data];
	}
	else if(state->type == LIGHT) {
		value = lookup_light[data];
	}
	else {
		debug("chrdev update: data error");
		return -EMEDIUMTYPE;
	}	
	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%03ld\n", value / 1000, value % 1000); 
	state->buf_timestamp = timestamp;
	
	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */

	int minor = iminor(inode);
	int num = minor/8;	
	int type = minor % 8;
	struct lunix_chrdev_state_struct* private_state_struct;
	int ret;
	
	debug("the sensor number is: %d\n", num);
	debug("the type of the sensor is: %d\n", type);
	debug("entering\n");
	ret = -ENODEV;

	/* nonseekable_open marks the given filp as being nonseekable (lseek cant be used here) */
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	
	/* Allocate a new Lunix character device private state structure */
	/* ? */
	
	private_state_struct = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(!private_state_struct) { 
		debug("read: kmalloc fail");
		ret =  -ENOMEM;
		goto out;
	}

	private_state_struct->type = type;
	private_state_struct->sensor = &lunix_sensors[num];
	private_state_struct->buf_lim = 0;
	private_state_struct->buf_timestamp = 0;
	sema_init(&private_state_struct->lock, 1);
	filp->private_data = private_state_struct;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	/* free any data allocated by open */
	kfree(filp->private_data);
	debug("device closed");
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret = 0;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock? */

	/* down_interruptible: while a user process waits here
	 * it can be interrupted by the user.
	 * Semaphore locks are used only in process context:
	 * Data that are accessed by an interrupt cant be locked
	 * with a semaphore because an interrupt cant sleep on a 
	 * wait queue
	 */
	if(down_interruptible(&state->lock))
		return -ERESTARTSYS;
	
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	if(*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* ? */
			up(&state->lock);	//release the lock


			debug("read: waiting for data to arrive\n");
			if(wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state))) {
				return -ERESTARTSYS;
			}
			if (down_interruptible(&state->lock)) 
				return -ERESTARTSYS;

			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
		}
	}
		
	/* End of file (??)*/
	/* ? */
	if(*f_pos >= state->buf_lim) {	
		debug("End of file");
		*f_pos = 0;
		goto out;
	}

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */
	if(*f_pos + cnt > state->buf_lim) 
		cnt = state->buf_lim - *f_pos;

	/* copy_to_user checks if the pointer passed by the userspace is valid, 
	 * and if its not, no copy is performed. if an invalid address is encountered
	 * during the copy only part of the data is copied and the function returns 
	 * the amount of memory still to be copied
	 */
	if(copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) { 
		ret =  -EFAULT;
		goto out;
	}
	*f_pos += cnt;
	ret = cnt;

	/* Auto-rewind on EOF mode? (??)*/
	/* ? */
	if(*f_pos == state->buf_lim)
		*f_pos = 0;

out:
	/* Unlock? */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0	
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
	
	debug("initializing character device\n");

	/* kernel uses cdev structs to represent char devices internally.
	 * Here we initialize a cdev struct with the lunix_chrdev file ops
	 * which is one of the fields of the struct
	 * */
	
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);

	/* cdev has an owner field that should be set to THIS_MODULE
	 */
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);	//this is the first of the device numbers we want to obtain
	
	/* register_chrdev_region? */
	/* this function obtains one or more device numbers */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix-chrdev");	
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}

	/* cdev_add? */
	/* cdev_add : Once the cdev struct is set up the next step is to tell the kernel about it */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;
		
	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
