/*
 *  mkbp.c - keyboard driver for Matrix KeyBoard Protocol keyboards.
 *
 *  Copyright (C) 2012 Google, Inc
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *
 * The MKBP (matrix keyboard protocol) is a message-based protocol for
 * communicating the keyboard state (which keys are pressed) from a keyboard EC
 * to the AP over some bus (such as i2c, lpc, spi).  The EC does debouncing,
 * but everything else (including deghosting) is done here.  The main
 * motivation for this is to keep the EC firmware as simple as possible, since
 * it cannot be easily upgraded and EC flash/IRAM space is relatively
 * expensive.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mfd/chromeos_ec.h>
#include <linux/mfd/chromeos_ec_commands.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

/*
 * The standard MKBP keyboard matrix table.
 *
 * These may become variables when we switch to the Device Tree. However, the
 * code and the protocol assume that NUM_ROWS = 8 (one byte per column).
 */
#define MKBP_NUM_ROWS 8
#define MKBP_NUM_COLS 13

struct mkbp_device {
	struct device *dev;
	struct input_dev *idev;
	struct chromeos_ec_device *ec;
	struct notifier_block notifier;
	struct notifier_block wake_notifier;
	uint8_t valid_keys[MKBP_NUM_COLS];
};

/* We will read this table from the Device Tree when we have one. */
static uint16_t mkbp_keycodes[MKBP_NUM_ROWS][MKBP_NUM_COLS] = {
	{ 0x0,		KEY_LEFTMETA,	KEY_F1,		KEY_B,
	  KEY_F10,	0x0,		KEY_N,		0x0,
	  KEY_EQUAL,	0x0,		KEY_RIGHTALT,	0x0,
	  0x0 },
	{ 0x0,		KEY_ESC,	KEY_F4,		KEY_G,
	  KEY_F7,	0x0,		KEY_H,		0x0,
	  KEY_APOSTROPHE,KEY_F9,	0x0,		KEY_BACKSPACE,
	  0x0 },
	{ KEY_LEFTCTRL,	KEY_TAB,	KEY_F3,		KEY_T,
	  KEY_F6,	KEY_RIGHTBRACE,	KEY_Y,		KEY_102ND,
	  KEY_LEFTBRACE,KEY_F8,		0x0,		0x0,
	  0x0 },
	{ 0x0,		KEY_GRAVE,	KEY_F2,		KEY_5,
	  KEY_F5,	0x0,		KEY_6,		0x0,
	  KEY_MINUS,	0x0,		0x0,		KEY_BACKSLASH,
	  0x0 },
	{ KEY_RIGHTCTRL,KEY_A,		KEY_D,		KEY_F,
	  KEY_S,	KEY_K,		KEY_J,		0x0,
	  KEY_SEMICOLON,KEY_L,		KEY_BACKSLASH,	KEY_ENTER,
	  0x0 },
	{ 0x0,		KEY_Z,		KEY_C,		KEY_V,
	  KEY_X,	KEY_COMMA,	KEY_M,		KEY_LEFTSHIFT,
	  KEY_SLASH,	KEY_DOT,	0x0,		KEY_SPACE,
	  0x0 },
	{ 0x0,		KEY_1,		KEY_3,		KEY_4,
	  KEY_2,	KEY_8,		KEY_7,		0x0,
	  KEY_0,	KEY_9,		KEY_LEFTALT,	KEY_DOWN,
	  KEY_RIGHT },
	{ KEY_BATTERY,	KEY_Q,		KEY_E,		KEY_R,
	  KEY_W,	KEY_I,		KEY_U,		KEY_RIGHTSHIFT,
	  KEY_P,	KEY_O,		0x0,		KEY_UP,
	  KEY_LEFT }
};

static uint8_t identity_keycodes[256];

/*
 * Sends a single key event to the input layer.
 */
static inline void mkbp_send_key_event(struct mkbp_device *mkbp_dev,
				int row, int col, int pressed)
{
	struct input_dev *idev = mkbp_dev->idev;
	int code = mkbp_keycodes[row][col];

	/* This key signifies a change to power supply status */
	if (code == KEY_BATTERY) {
		if (mkbp_dev->ec->charger)
			power_supply_changed(mkbp_dev->ec->charger);
		return;
	}

	input_report_key(idev, code, pressed);
}

/*
 * Returns true when there is at least one combination of pressed keys that
 * results in ghosting.
 */
static bool mkbp_has_ghosting(struct mkbp_device *mkbp_dev, uint8_t *buf)
{
	int col, row;
	int mask, mask_corner;
	int pressed_in_row[MKBP_NUM_ROWS];
	int pressed_in_col[MKBP_NUM_COLS];
	struct device *dev = mkbp_dev->dev;
	uint8_t *valid_keys = mkbp_dev->valid_keys;
	int max_in_row = 0;
	int max_in_col = 0;
	int n_corners = 0;
	struct {
		int row, col;
	} corners[10];
	int col_corner, row_corner;

	memset(pressed_in_row, 0, sizeof(pressed_in_row));
	memset(pressed_in_col, 0, sizeof(pressed_in_col));
	/*
	 * Ghosting happens if for any pressed key X there are other keys

	 * pressed both in the same row and column of X as, for instance,
	 * in the following diagram:
	 *
	 * . . Y . g .
	 * . . . . . .
	 * . . . . . .
	 * . . X . Z .
	 *
	 * In this case only X, Y, and Z are pressed, but g appears to be
	 * pressed too (see Wikipedia).
	 */
	for (row = 0; row < MKBP_NUM_ROWS; row++) {
		mask = 1 << row;
		for (col = 0; col < MKBP_NUM_COLS; col++) {
			if (mask & buf[col] & valid_keys[col]) {
				pressed_in_row[row] += 1;
				pressed_in_col[col] += 1;
				if (pressed_in_col[col] > max_in_col)
					max_in_col = pressed_in_col[col];
			}
		}
		if (pressed_in_row[row] > max_in_row)
			max_in_row = pressed_in_row[row];
	}

	if (max_in_col < 2 || max_in_row < 2)
		return false;

       /* Find possible ghosting locations.  These are the corners of the L's.
	* We know there is at least one L (i.e. one point whose row has at
	* least two keys ON and whose column has at least two keys ON).
	*/
	for (row = 0; row < MKBP_NUM_ROWS; row++) {
		mask = 1 << row;
		if (pressed_in_row[row] < 2)
			continue;
		for (col = 0; col < MKBP_NUM_COLS; col++) {
			if (pressed_in_col[col] < 2)
				continue;
			if (buf[col] & mask & valid_keys[col]) {
				corners[n_corners].row = row;
				corners[n_corners].col = col;
				n_corners++;
				if (n_corners == sizeof(corners) /
				    sizeof(corners[0])) {
					/* give up */
					dev_dbg(dev, "too many corners!");
					return true;
				}
			}
		}
	}
	/* Examine all corners for possible ghosting. */
	for (n_corners--; n_corners >= 0; n_corners--) {
		row_corner = corners[n_corners].row;
		col_corner = corners[n_corners].col;
		mask_corner = 1 << row_corner;
		/* Find the other bits in this column. */
		for (row = 0; row < MKBP_NUM_ROWS; row++) {
			if (row == row_corner)
				/* Skip the corner. */
				continue;
			mask = 1 << row;
			if (!(buf[col_corner] & mask & valid_keys[col_corner]))
				/* Key is OFF */
				continue;
			/* [row, col_corner] is ON.  Find the other bits in
			 * row_corner.
			 */
			for (col = 0; col < MKBP_NUM_COLS; col++) {
				if (col == col_corner)
					/* Skip the corner. */
					continue;
				if (!(buf[col] & mask_corner))
					/* Key is OFF. */
					continue;
				/* If we get here, [row_corner, col] is ON,
				 * therefore [row, col] is the possible
				 * ghosting location (diagonally opposite).  If
				 * that key is wired, we have ghosting.
				 */
				if (valid_keys[col] & mask) {
					dev_dbg(dev, "ghost found at: r%d c%d,"
						" corners: r:0x%x c:0x%x\n",
						row, col, row_corner,
						col_corner);
					return true;
				}
			}
		}
	}
	return false;
}

/*
 * mkbp_old_state[row][col] is 1 when the most recent (valid) communication
 * with the keyboard indicated that the key at row/col was in the pressed
 * state.
 */
static uint8_t mkbp_old_state[MKBP_NUM_ROWS][MKBP_NUM_COLS];

/*
 * Compares the new keyboard state to the old one and produces key
 * press/release events accordingly.  The keyboard state is 13 bytes (one byte
 * per column)
 */
static void mkbp_process(struct mkbp_device *mkbp_dev,
			 uint8_t *kb_state, int len)
{
	int col, row;
	int new_state;
	int num_cols;

	num_cols = len;

	if (mkbp_has_ghosting(mkbp_dev, kb_state)) {
		/*
		 * Simple-minded solution: ignore this state. The obvious
		 * improvement is to only ignore changes to keys involved in
		 * the ghosting, but process the other changes.
		 */
		dev_dbg(mkbp_dev->dev, "ghosting found\n");
		return;
	}

	for (col = 0; col < MKBP_NUM_COLS; col++) {
		for (row = 0; row < MKBP_NUM_ROWS; row++) {
			new_state = kb_state[col] & (1 << row);
			if (!!new_state != mkbp_old_state[row][col]) {
			  dev_dbg(mkbp_dev->dev,
				  "changed: [r%d c%d]: byte %02x\n",
				  row, col, new_state);
			}
			if (new_state && !mkbp_old_state[row][col]) {
				/* key press */
				mkbp_send_key_event(mkbp_dev, row, col, 1);
				mkbp_old_state[row][col] = 1;
			} else if (!new_state && mkbp_old_state[row][col]) {
				/* key release */
				mkbp_send_key_event(mkbp_dev, row, col, 0);
				mkbp_old_state[row][col] = 0;
			}
		}
	}
	input_sync(mkbp_dev->idev);
}

static int mkbp_open(struct input_dev *dev)
{
	struct mkbp_device *mkbp_dev = input_get_drvdata(dev);
	int ret;

	ret = blocking_notifier_chain_register(&mkbp_dev->ec->event_notifier,
						&mkbp_dev->notifier);
	if (ret)
		return ret;
	ret = blocking_notifier_chain_register(&mkbp_dev->ec->wake_notifier,
						&mkbp_dev->wake_notifier);
	if (ret) {
		blocking_notifier_chain_unregister(
			&mkbp_dev->ec->event_notifier, &mkbp_dev->notifier);
		return ret;
	}

	return 0;
}

static void mkbp_close(struct input_dev *dev)
{
	struct mkbp_device *mkbp_dev = input_get_drvdata(dev);

	blocking_notifier_chain_unregister(&mkbp_dev->ec->event_notifier,
					   &mkbp_dev->notifier);
	blocking_notifier_chain_unregister(&mkbp_dev->ec->wake_notifier,
					   &mkbp_dev->wake_notifier);
}

static int mkbp_get_state(struct mkbp_device *mkbp_dev, uint8_t *kb_state)
{
	return mkbp_dev->ec->command_recv(mkbp_dev->ec, EC_CMD_MKBP_STATE,
					  kb_state, MKBP_NUM_COLS);
}

static int mkbp_work(struct notifier_block *nb,
		     unsigned long state, void *_notify)
{
	int ret;
	struct mkbp_device *mkbp_dev = container_of(nb, struct mkbp_device,
						    notifier);
	uint8_t kb_state[MKBP_NUM_COLS];

	ret = mkbp_get_state(mkbp_dev, kb_state);
	if (ret >= 0)
		mkbp_process(mkbp_dev, kb_state, ret);

	return NOTIFY_DONE;
}

/* On resume, clear any keys in the buffer, crosbug.com/p/14523 */
static int mkbp_clear_keyboard(struct notifier_block *nb,
			       unsigned long state, void *_notify)
{
	struct mkbp_device *mkbp_dev = container_of(nb, struct mkbp_device,
						    wake_notifier);
	uint8_t old_state[MKBP_NUM_COLS];
	uint8_t new_state[MKBP_NUM_COLS];
	unsigned long duration;
	int i, ret;

	/*
	 * Keep reading until we see that the scan state does not change.
	 * That indicates that we are done.
	 *
	 * Assume that the EC keyscan buffer is at most 32 deep.
	 *
	 * TODO(sjg@chromium.org): Add EC command to clear keyscan FIFO.
	 */
	duration = jiffies;
	ret = mkbp_get_state(mkbp_dev, new_state);
	for (i = 1; !ret && i < 32; i++) {
		memcpy(old_state, new_state, sizeof(old_state));
		ret = mkbp_get_state(mkbp_dev, new_state);
		if (0 == memcmp(old_state, new_state, sizeof(old_state)))
			break;
	}
	duration = jiffies - duration;
	dev_info(mkbp_dev->dev, "Discarded %d keyscan(s) in %dus\n", i,
		jiffies_to_usecs(duration));

	return 0;
}

/*
 * Walks keycodes flipping bit in buffer COLUMNS deep where bit is ROW.  Used by
 * ghosting logic to ignore NULL or virtual keys.
 */
static void __devinit mkbp_compute_valid_keys(struct mkbp_device *mkbp_dev)
{
	int row, col;
	uint16_t code;

	BUILD_BUG_ON(MKBP_NUM_ROWS > sizeof(mkbp_dev->valid_keys));

	for (col = 0; col < MKBP_NUM_COLS; col++) {
		for (row = 0; row < MKBP_NUM_ROWS; row++) {
			code = mkbp_keycodes[row][col];
			if (code && (code != KEY_BATTERY))
				mkbp_dev->valid_keys[col] |= 1 << row;
		}
		dev_dbg(mkbp_dev->dev, "valid_keys[%02d] = 0x%02x\n",
			col, mkbp_dev->valid_keys[col]);
	}
}

static int __devinit mkbp_probe(struct platform_device *pdev)
{
	struct chromeos_ec_device *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = ec->dev;
	struct mkbp_device *mkbp_dev = NULL;
	struct input_dev *idev = NULL;
	int i, err;
	bool input_device_registered = false;

	dev_dbg(dev, "probing\n");

	mkbp_dev = kzalloc(sizeof(*mkbp_dev), GFP_KERNEL);
	idev = input_allocate_device();
	if (idev == NULL || mkbp_dev == NULL) {
		err = -ENOMEM;
		dev_err(dev, "cannot allocate\n");
		goto fail;
	}

	mkbp_dev->ec = ec;
	mkbp_dev->notifier.notifier_call = mkbp_work;
	mkbp_dev->wake_notifier.notifier_call = mkbp_clear_keyboard;
	mkbp_dev->dev = dev;
	mkbp_compute_valid_keys(mkbp_dev);

	idev->name = ec->get_name(ec);
	idev->phys = ec->get_phys_name(ec);
	idev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
	idev->keycode = identity_keycodes;
	idev->keycodesize = sizeof(identity_keycodes[0]);
	idev->keycodemax =
		sizeof(identity_keycodes) / sizeof(identity_keycodes[0]);
	for (i = 0; i < idev->keycodemax; i++) {
		identity_keycodes[i] = i;
		input_set_capability(idev, EV_KEY, i);
	}

	/* TODO(sjg@chromium.org): This could be SPI or LPC */
	idev->id.bustype = BUS_I2C;
	idev->id.version = 1;
	idev->id.product = 0;
	idev->dev.parent = ec->get_parent(ec);
	idev->open = mkbp_open;
	idev->close = mkbp_close;

	input_set_drvdata(idev, mkbp_dev);
	mkbp_dev->idev = idev;
	err = input_register_device(mkbp_dev->idev);
	if (err) {
		dev_err(dev, "cannot register input device\n");
		goto fail;
	}
	/* We have seen the mkbp work function scheduled as much as 300ms after
	 * the interrupt service routine is called.  The default autorepeat
	 * delay is 250ms.  This can lead to spurious autorepeat.  A better fix
	 * would be to collect time stamps in the ISR, but for the moment a
	 * longer delay helps.
	 *
	 * Also note that we must change the delay after device registration,
	 * or else the input layer assumes that the driver does its own
	 * autorepeat.  (Which we will probably have to do.)
	 */
	mkbp_dev->idev->rep[REP_DELAY] = 600;
	input_device_registered = true;

	dev_info(dev, "MKBP Keyboard ready\n");

	return err;
fail:
	if (input_device_registered)
		input_unregister_device(idev);
	kfree(mkbp_dev);
	input_free_device(idev);
	return err;
}

static struct platform_driver mkbp_driver = {
	.probe = mkbp_probe,
	.driver = {
		.name = "mkbp",
	},
};


module_platform_driver(mkbp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Matrix keyboard protocol driver");
MODULE_ALIAS("platform:mkbp");
