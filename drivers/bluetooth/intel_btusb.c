/*
 *
 *  Intel Bluetooth USB driver for Chrome OS (before 3.10)
 *
 *  Copyright (C) 2005-2008  Marcel Holtmann <marcel@holtmann.org>
 *
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
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/firmware.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define VERSION "0.6"

static struct usb_driver intel_btusb_driver;

static struct usb_device_id intel_btusb_table[] = {
	/* Intel Bluetooth 7260 device */
	{ USB_DEVICE(0x8087, 0x07dc) },

	{ }	/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, intel_btusb_table);

#define BTUSB_MAX_ISOC_FRAMES	10

#define BTUSB_INTR_RUNNING	0
#define BTUSB_BULK_RUNNING	1
#define BTUSB_ISOC_RUNNING	2
#define BTUSB_SUSPENDING	3
#define BTUSB_DID_ISO_RESUME	4

#define MAX_DATA_SIZE	260
#define CMD_TIMEOUT	msecs_to_jiffies(2000)	/* 2 seconds */

struct intel_btusb_data {
	struct hci_dev       *hdev;
	struct usb_device    *udev;
	struct usb_interface *intf;
	struct usb_interface *isoc;

	spinlock_t lock;

	unsigned long flags;

	struct work_struct work;
	struct work_struct waker;

	struct usb_anchor tx_anchor;
	struct usb_anchor intr_anchor;
	struct usb_anchor bulk_anchor;
	struct usb_anchor isoc_anchor;
	struct usb_anchor deferred;
	int tx_in_flight;
	spinlock_t txlock;

	struct usb_endpoint_descriptor *intr_ep;
	struct usb_endpoint_descriptor *bulk_tx_ep;
	struct usb_endpoint_descriptor *bulk_rx_ep;
	struct usb_endpoint_descriptor *isoc_tx_ep;
	struct usb_endpoint_descriptor *isoc_rx_ep;

	__u8 cmdreq_type;

	unsigned int sco_num;
	int isoc_altsetting;
	int suspend_count;

	/* Intel specific data */
	u8 evt_buff[MAX_DATA_SIZE];
	int evt_len;
	int patched;
};

struct intel_version {
	u8 status;
	u8 hw_platform;
	u8 hw_variant;
	u8 hw_revision;
	u8 fw_variant;
	u8 fw_revision;
	u8 fw_build_num;
	u8 fw_build_ww;
	u8 fw_build_yy;
	u8 fw_patch_num;
} __packed;

static int intel_btusb_send_cmd_sync_ev(struct intel_btusb_data *data,
					const u8 *cmd_buff, int cmd_len,
					u8 event, u32 timeout)
{
	u8 *buff;
	int err, next = 1;

	BT_DBG("send cmd len %d", cmd_len);

	buff = kmalloc(cmd_len, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	memcpy(buff, cmd_buff, cmd_len);

	err = usb_control_msg(data->udev, usb_sndctrlpipe(data->udev, 0x00),
			0, USB_TYPE_CLASS, 0, 0,
			buff, cmd_len, USB_CTRL_SET_TIMEOUT);
	if (err < 0) {
		BT_ERR("failed to send control msg (%d)", err);
		goto exit_error;
	}

	while (next) {
		err = usb_interrupt_msg(data->udev,
					usb_rcvintpipe(data->udev, 0x81),
					data->evt_buff, MAX_DATA_SIZE,
					&data->evt_len, 5000);
		if (err < 0) {
			BT_ERR("failed to receive interrupt msg (%d)", err);
			goto exit_error;
		}

		BT_DBG("received evt (%d): %02x %02x %02x %02x %02x %02x",
		       data->evt_len, data->evt_buff[0], data->evt_buff[1],
		       data->evt_buff[2], data->evt_buff[3], data->evt_buff[4],
		       data->evt_buff[5]);

		/*
		 * Stop reading the event if event parameter is 0 or received
		 * expected event code.
		 */
		if (!event || event == data->evt_buff[0])
			goto exit_error;
	}

exit_error:
	kfree(buff);

	return err;
}

static int intel_btusb_send_cmd_sync(struct intel_btusb_data *data,
					const u8 *cmd_buff, int cmd_len,
					u32 timeout)
{
	return intel_btusb_send_cmd_sync_ev(data, cmd_buff, cmd_len, 0,
								timeout);
}


static const struct firmware *intel_btusb_get_fw(struct intel_btusb_data *data,
						struct intel_version *ver)
{
	const struct firmware *fw;
	char fwname[64];
	int err;

	snprintf(fwname, sizeof(fwname),
		 "intel/ibt-hw-%x.%x.%x-fw-%x.%x.%x.%x.%x.bseq",
		 ver->hw_platform, ver->hw_variant, ver->hw_revision,
		 ver->fw_variant,  ver->fw_revision, ver->fw_build_num,
		 ver->fw_build_ww, ver->fw_build_yy);

	BT_INFO("Intel Bluetooth firmware file: %s", fwname);

	err = request_firmware(&fw, fwname, &data->udev->dev);
	if (err < 0) {
		if (err == -EINVAL) {
			BT_ERR("Intel firmware request failed (%d)", err);
			return NULL;
		}

		BT_ERR("failed to open Intel firmware: %s(%d)", fwname, err);

		/*
		 * If the correct firmware patch file is not found, use the
		 * default firmware patch file instead
		 */
		snprintf(fwname, sizeof(fwname), "intel/ibt-hw-%x.%x.bseq",
			 ver->hw_platform, ver->hw_variant);
		err = request_firmware(&fw, fwname, &data->udev->dev);
		if (err < 0) {
			BT_ERR("failed to open default Intel firmware: %s(%d)",
			       fwname, err);
			return NULL;
		}

		BT_INFO("Using default Intel Bluetooth firmware: %s", fwname);
	}

	return fw;
}

static int intel_btusb_patching(struct intel_btusb_data *data,
				      const struct firmware *fw,
				      const u8 **fw_ptr, int *disable_patch)
{
	struct hci_command_hdr *cmd;
	const u8 *cmd_ptr;
	struct hci_event_hdr *evt = NULL;
	const u8 *evt_ptr = NULL;
	int err;
	int remain = fw->size - (*fw_ptr - fw->data);

	/* The first byte indicates the types of the patch command or event.
	 * 0x01 means HCI command and 0x02 is expected HCI event.
	 * If the first byte in the current firmware buffer doesn't start
	 * with 0x01 or the size of remain buffer is smaller than HCI command
	 * header, the firmware file is corrupted and it should stop
	 * the patching process and exit.
	 */
	BT_DBG("remain fw data: %d", remain);

	if (remain <= HCI_COMMAND_HDR_SIZE || *fw_ptr[0] != 0x01) {
		BT_ERR("fw file is corrupted: invalid cmd");
		return -EINVAL;
	}
	(*fw_ptr)++;
	remain--;

	cmd = (struct hci_command_hdr *)(*fw_ptr);
	cmd_ptr = *fw_ptr;
	*fw_ptr += sizeof(*cmd);
	remain -= sizeof(*cmd);

	/* Ensure that the remain firmware data is long enough than the length
	 * of command parameter. If not, the firmware is corrupted.
	 */
	if (remain < cmd->plen) {
		BT_ERR("fw file is corrupted. invalid cmd len");
		return -EFAULT;
	}

	/* If a specific command is presented in the firmware file,
	 * then enable the patch once downloading is completed with success.
	 * Otherwise disable the patch when exit the manufacturer mode.
	 */
	if (*disable_patch && le16_to_cpu(cmd->opcode) == 0xfc8e)
		*disable_patch = 0;

	*fw_ptr += cmd->plen;
	remain -= cmd->plen;

	/* This reads the expected events when the above command is sent to the
	 * device. Some vendor commands expects more than one events,
	 * for example command status event followed by vendor specific event.
	 * In this case, it only keeps the last expected event. so the command
	 * can be sent with intel_btusb_send_cmd_sync_ev() which returns the
	 * sk_buff of last expected event.
	 */
	while (remain > HCI_EVENT_HDR_SIZE && *fw_ptr[0] == 0x02) {
		(*fw_ptr)++;
		remain--;

		evt = (struct hci_event_hdr *)(*fw_ptr);
		evt_ptr = *fw_ptr;

		/* push HCI event header */
		*fw_ptr += sizeof(*evt);
		remain -= sizeof(*evt);

		if (remain < evt->plen) {
			BT_ERR("fw file is corrupted: invalid evt len");
			return -EFAULT;
		}

		*fw_ptr += evt->plen;
		remain -= evt->plen;
	}

	/* Every HCI commands in the firmware file has its correspond event.
	 * If event is not found or remain is smaller than zero, the firmware
	 * file is corrupted.
	 */
	if (!evt || !evt_ptr || remain < 0) {
		BT_ERR("Intel fw corrupted: invalid evt read");
		return -EFAULT;
	}

	err = intel_btusb_send_cmd_sync_ev(data, cmd_ptr, cmd->plen + 3,
					evt->evt, CMD_TIMEOUT);
	if (err) {
		BT_ERR("failed to send patch command (0x%4.4x) failed(%d)",
		       cmd->opcode, err);
		return -EFAULT;
	}

	/* It ensures that the returned event matches the event data read from
	 * the firmware file. At fist, it checks the length and then
	 * the contents of the event.
	 */
	if (evt->plen + 2 != data->evt_len) {
		BT_ERR("mismatch event length (opcode 0x%4.4x)", cmd->opcode);
		return -EFAULT;
	}

	if (memcmp(data->evt_buff, evt_ptr, evt->plen + 2)) {
		BT_ERR("mismatch event parameter (opcode 0x%4.4x)",
		       cmd->opcode);
		return -EFAULT;
	}

	return 0;
}

static int intel_fw_setup(struct intel_btusb_data *data)
{
	const struct firmware *fw;
	const u8 *fw_ptr;
	struct intel_version *ver;
	int err;
	int disable_patch;

	const u8 get_version[] = { 0x05, 0xFC, 0x00 };
	const u8 mfg_enable[] = { 0x11, 0xFC, 0x02, 0x01, 0x00 };
	const u8 mfg_disable[] = { 0x11, 0xFC, 0x02, 0x00, 0x00 };
	const u8 mfg_reset_deactivate[] = { 0x11, 0xFC, 0x02, 0x00, 0x01 };
	const u8 mfg_reset_activate[] = { 0x11, 0xFC, 0x02, 0x00, 0x02 };

	BT_INFO("Start Intel fw setup: data %p", data);

	/* Read Intel specific controller version first to allow selection of
	 * which firmware file to load.
	 *
	 * The returned information are hardware variant and revision plus
	 * firmware variant, revision and build number.
	 */
	err = intel_btusb_send_cmd_sync(data, get_version, 3, CMD_TIMEOUT);
	if (err < 0) {
		BT_ERR("reading Intel fw version command failed (%d)", err);
		return err;
	}

	if (data->evt_len != 15) {
		BT_ERR("Intel version event len mismatch: (%d)", data->evt_len);
		return -EIO;
	}

	ver = (struct intel_version *)&data->evt_buff[5];
	if (ver->status) {
		BT_ERR("Intel fw version event failed (%02x)", ver->status);
		return -bt_to_errno(ver->status);
	}

	BT_INFO("Intel fw version: %02x%02x%02x%02x%02x%02x%02x%02x%02x",
		ver->hw_platform, ver->hw_variant, ver->hw_revision,
		ver->fw_variant,  ver->fw_revision, ver->fw_build_num,
		ver->fw_build_ww, ver->fw_build_yy, ver->fw_patch_num);

	/* fw_patch_num indicates the version of patch the device currently
	 * have. If there is no patch data in the device, it is always 0x00.
	 * So, if it is other than 0x00, no need to patch the deivce again.
	 */
	if (ver->fw_patch_num) {
		BT_INFO("Intel device is already patched. patch version: %02x",
			ver->fw_patch_num);
		return 0;
	}

	/* If the device is Intel Bluetooth Bootloader device, patching is not
	 * required.
	 */
	if (ver->hw_platform == 0x20) {
		BT_INFO("This is Intel Bluetooth Bootloader device. Skip...");
		return 0;
	}

	/* Opens the firmware patch file based on the firmware version read
	 * from the controller. If it fails to open the matching firmware
	 * patch file, it tries to open the default firmware patch file.
	 * If no patch file is found, allow the device to operate without
	 * a patch.
	 */
	fw = intel_btusb_get_fw(data, ver);
	if (!fw)
		return -ENOENT;

	fw_ptr = fw->data;

	/* This Intel specific command enables the manufacturer mode of the
	 * controller.
	 *
	 * Only while this mode is enabled, the driver can download the
	 * firmware patch data and configuration parameters.
	 */
	err = intel_btusb_send_cmd_sync(data, mfg_enable, 5, CMD_TIMEOUT);
	if (err < 0) {
		BT_ERR("enabling Intel manufacturer mode failed (%d)", err);
		release_firmware(fw);
		return err;
	}

	if (data->evt_buff[5]) {
		BT_ERR("enable Intel manufacturer mode event failed (%02x)",
		       data->evt_buff[5]);
		release_firmware(fw);
		return -bt_to_errno(data->evt_buff[5]);
	}

	disable_patch = 1;

	/* The firmware data file consists of list of Intel specific HCI
	 * commands and its expected events. The first byte indicates the
	 * type of the message, either HCI command or HCI event.
	 *
	 * It reads the command and its expected event from the firmware file,
	 * and send to the controller. Once __hci_cmd_sync_ev() returns,
	 * the returned event is compared with the event read from the firmware
	 * file and it will continue until all the messages are downloaded to
	 * the controller.
	 *
	 * Once the firmware patching is completed successfully,
	 * the manufacturer mode is disabled with reset and activating the
	 * downloaded patch.
	 *
	 * If the firmware patching fails, the manufacturer mode is
	 * disabled with reset and deactivating the patch.
	 *
	 * If the default patch file is used, no reset is done when disabling
	 * the manufacturer.
	 */
	while (fw->size > fw_ptr - fw->data) {
		err = intel_btusb_patching(data, fw, &fw_ptr, &disable_patch);
		if (err < 0)
			goto exit_mfg_deactivate;
	}

	release_firmware(fw);

	if (disable_patch)
		goto exit_mfg_disable;

	/* Patching completed successfully and disable the manufacturer mode
	 * with reset and activate the downloaded firmware patches.
	 */
	err = intel_btusb_send_cmd_sync(data, mfg_reset_activate, 5,
					CMD_TIMEOUT);
	if (err < 0) {
		BT_ERR("exiting Intel manufacturer mode failed(activate):(%d)",
		       err);
		return err;
	}

	BT_INFO("Intel Bluetooth firmware patch completed and activated");

	return 0;

exit_mfg_disable:
	/* Disable the manufacturer mode without reset */
	err = intel_btusb_send_cmd_sync(data, mfg_disable, 5, CMD_TIMEOUT);
	if (err < 0) {
		BT_ERR("exiting Intel manufacturer mode failed(disable): (%d)",
		       err);
		return err;
	}

	BT_INFO("Intel Bluetooth firmware patch completed and disabled");

	return 0;

exit_mfg_deactivate:
	release_firmware(fw);

	/* Patching failed. Disable the manufacturer mode with reset and
	 * don't enable the firmware patches.
	 */
	err = intel_btusb_send_cmd_sync(data, mfg_reset_deactivate, 5,
					CMD_TIMEOUT);
	if (err < 0) {
		BT_ERR("exiting Intel manufacturer mode failed(deact): (%d)",
		       err);
		return err;
	}

	BT_INFO("Intel Bluetooth firmware patch completed and deactivated");

	return 0;
}


static int intel_inc_tx(struct intel_btusb_data *data)
{
	unsigned long flags;
	int rv;

	spin_lock_irqsave(&data->txlock, flags);
	rv = test_bit(BTUSB_SUSPENDING, &data->flags);
	if (!rv)
		data->tx_in_flight++;
	spin_unlock_irqrestore(&data->txlock, flags);

	return rv;
}

static void intel_btusb_intr_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	int err;

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

		if (hci_recv_fragment(hdev, HCI_EVENT_PKT,
						urb->transfer_buffer,
						urb->actual_length) < 0) {
			BT_ERR("%s corrupted event packet", hdev->name);
			hdev->stat.err_rx++;
		}
	}

	if (!test_bit(BTUSB_INTR_RUNNING, &data->flags))
		return;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int intel_btusb_submit_intr_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BT_DBG("%s", hdev->name);

	if (!data->intr_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->intr_ep->wMaxPacketSize);

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvintpipe(data->udev, data->intr_ep->bEndpointAddress);

	usb_fill_int_urb(urb, data->udev, pipe, buf, size,
				intel_btusb_intr_complete, hdev,
				data->intr_ep->bInterval);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_anchor_urb(urb, &data->intr_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void intel_btusb_bulk_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	int err;

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		hdev->stat.byte_rx += urb->actual_length;

		if (hci_recv_fragment(hdev, HCI_ACLDATA_PKT,
						urb->transfer_buffer,
						urb->actual_length) < 0) {
			BT_ERR("%s corrupted ACL packet", hdev->name);
			hdev->stat.err_rx++;
		}
	}

	if (!test_bit(BTUSB_BULK_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->bulk_anchor);
	usb_mark_last_busy(data->udev);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static int intel_btusb_submit_bulk_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size = HCI_MAX_FRAME_SIZE;

	BT_DBG("%s", hdev->name);

	if (!data->bulk_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(0, mem_flags);
	if (!urb)
		return -ENOMEM;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvbulkpipe(data->udev, data->bulk_rx_ep->bEndpointAddress);

	usb_fill_bulk_urb(urb, data->udev, pipe,
				buf, size, intel_btusb_bulk_complete, hdev);

	urb->transfer_flags |= URB_FREE_BUFFER;

	usb_mark_last_busy(data->udev);
	usb_anchor_urb(urb, &data->bulk_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void intel_btusb_isoc_complete(struct urb *urb)
{
	struct hci_dev *hdev = urb->context;
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	int i, err;

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return;

	if (urb->status == 0) {
		for (i = 0; i < urb->number_of_packets; i++) {
			unsigned int offset = urb->iso_frame_desc[i].offset;
			unsigned int length = urb->iso_frame_desc[i].actual_length;

			if (urb->iso_frame_desc[i].status)
				continue;

			hdev->stat.byte_rx += length;

			if (hci_recv_fragment(hdev, HCI_SCODATA_PKT,
						urb->transfer_buffer + offset,
						length) < 0) {
				BT_ERR("%s corrupted SCO packet", hdev->name);
				hdev->stat.err_rx++;
			}
		}
	}

	if (!test_bit(BTUSB_ISOC_RUNNING, &data->flags))
		return;

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		/* -EPERM: urb is being killed;
		 * -ENODEV: device got disconnected */
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p failed to resubmit (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}
}

static inline void __intel_fill_isoc_descriptor(struct urb *urb, int len,
								int mtu)
{
	int i, offset = 0;

	BT_DBG("len %d mtu %d", len, mtu);

	for (i = 0; i < BTUSB_MAX_ISOC_FRAMES && len >= mtu;
					i++, offset += mtu, len -= mtu) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = mtu;
	}

	if (len && i < BTUSB_MAX_ISOC_FRAMES) {
		urb->iso_frame_desc[i].offset = offset;
		urb->iso_frame_desc[i].length = len;
		i++;
	}

	urb->number_of_packets = i;
}

static int intel_btusb_submit_isoc_urb(struct hci_dev *hdev, gfp_t mem_flags)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	struct urb *urb;
	unsigned char *buf;
	unsigned int pipe;
	int err, size;

	BT_DBG("%s", hdev->name);

	if (!data->isoc_rx_ep)
		return -ENODEV;

	urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, mem_flags);
	if (!urb)
		return -ENOMEM;

	size = le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize) *
						BTUSB_MAX_ISOC_FRAMES;

	buf = kmalloc(size, mem_flags);
	if (!buf) {
		usb_free_urb(urb);
		return -ENOMEM;
	}

	pipe = usb_rcvisocpipe(data->udev, data->isoc_rx_ep->bEndpointAddress);

	usb_fill_int_urb(urb, data->udev, pipe, buf, size,
				intel_btusb_isoc_complete,
				hdev, data->isoc_rx_ep->bInterval);

	urb->transfer_flags  = URB_FREE_BUFFER | URB_ISO_ASAP;

	__intel_fill_isoc_descriptor(urb, size,
			le16_to_cpu(data->isoc_rx_ep->wMaxPacketSize));

	usb_anchor_urb(urb, &data->isoc_anchor);

	err = usb_submit_urb(urb, mem_flags);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		usb_unanchor_urb(urb);
	}

	usb_free_urb(urb);

	return err;
}

static void intel_btusb_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *) skb->dev;
	struct intel_btusb_data *data = hci_get_drvdata(hdev);

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	spin_lock(&data->txlock);
	data->tx_in_flight--;
	spin_unlock(&data->txlock);

	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static void intel_btusb_isoc_tx_complete(struct urb *urb)
{
	struct sk_buff *skb = urb->context;
	struct hci_dev *hdev = (struct hci_dev *) skb->dev;

	BT_DBG("%s urb %p status %d count %d", hdev->name,
					urb, urb->status, urb->actual_length);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (!urb->status)
		hdev->stat.byte_tx += urb->transfer_buffer_length;
	else
		hdev->stat.err_tx++;

done:
	kfree(urb->setup_packet);

	kfree_skb(skb);
}

static int intel_btusb_open(struct hci_dev *hdev)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	int err;

	BT_DBG("%s", hdev->name);

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return err;

	data->intf->needs_remote_wakeup = 1;

	if (test_and_set_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (test_and_set_bit(BTUSB_INTR_RUNNING, &data->flags))
		goto done;

	if (!data->patched) {
		err = intel_fw_setup(data);
		if (err < 0)
			return err;
		data->patched = 1;
	}

	err = intel_btusb_submit_intr_urb(hdev, GFP_KERNEL);
	if (err < 0)
		goto failed;

	err = intel_btusb_submit_bulk_urb(hdev, GFP_KERNEL);
	if (err < 0) {
		usb_kill_anchored_urbs(&data->intr_anchor);
		goto failed;
	}

	set_bit(BTUSB_BULK_RUNNING, &data->flags);
	intel_btusb_submit_bulk_urb(hdev, GFP_KERNEL);

done:
	usb_autopm_put_interface(data->intf);
	return 0;

failed:
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);
	clear_bit(HCI_RUNNING, &hdev->flags);
	usb_autopm_put_interface(data->intf);
	return err;
}

static void intel_btusb_stop_traffic(struct intel_btusb_data *data)
{
	usb_kill_anchored_urbs(&data->intr_anchor);
	usb_kill_anchored_urbs(&data->bulk_anchor);
	usb_kill_anchored_urbs(&data->isoc_anchor);
}

static int intel_btusb_close(struct hci_dev *hdev)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	int err;

	BT_DBG("%s", hdev->name);

	if (!test_and_clear_bit(HCI_RUNNING, &hdev->flags))
		return 0;

	cancel_work_sync(&data->work);
	cancel_work_sync(&data->waker);

	clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
	clear_bit(BTUSB_BULK_RUNNING, &data->flags);
	clear_bit(BTUSB_INTR_RUNNING, &data->flags);

	intel_btusb_stop_traffic(data);
	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		goto failed;

	data->intf->needs_remote_wakeup = 0;
	usb_autopm_put_interface(data->intf);

failed:
	usb_scuttle_anchored_urbs(&data->deferred);
	return 0;
}

static int intel_btusb_flush(struct hci_dev *hdev)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);

	BT_DBG("%s", hdev->name);

	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

static int intel_btusb_send_frame(struct sk_buff *skb)
{
	struct hci_dev *hdev = (struct hci_dev *) skb->dev;
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	struct usb_ctrlrequest *dr;
	struct urb *urb;
	unsigned int pipe;
	int err;

	BT_DBG("%s", hdev->name);

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		return -EBUSY;

	switch (bt_cb(skb)->pkt_type) {
	case HCI_COMMAND_PKT:
		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		dr = kmalloc(sizeof(*dr), GFP_ATOMIC);
		if (!dr) {
			usb_free_urb(urb);
			return -ENOMEM;
		}

		dr->bRequestType = data->cmdreq_type;
		dr->bRequest     = 0;
		dr->wIndex       = 0;
		dr->wValue       = 0;
		dr->wLength      = __cpu_to_le16(skb->len);

		pipe = usb_sndctrlpipe(data->udev, 0x00);

		usb_fill_control_urb(urb, data->udev, pipe, (void *) dr,
						skb->data, skb->len,
						intel_btusb_tx_complete, skb);

		hdev->stat.cmd_tx++;
		break;

	case HCI_ACLDATA_PKT:
		if (!data->bulk_tx_ep)
			return -ENODEV;

		urb = usb_alloc_urb(0, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		pipe = usb_sndbulkpipe(data->udev,
					data->bulk_tx_ep->bEndpointAddress);

		usb_fill_bulk_urb(urb, data->udev, pipe,
						skb->data, skb->len,
						intel_btusb_tx_complete, skb);

		hdev->stat.acl_tx++;
		break;

	case HCI_SCODATA_PKT:
		if (!data->isoc_tx_ep || hdev->conn_hash.sco_num < 1)
			return -ENODEV;

		urb = usb_alloc_urb(BTUSB_MAX_ISOC_FRAMES, GFP_ATOMIC);
		if (!urb)
			return -ENOMEM;

		pipe = usb_sndisocpipe(data->udev,
					data->isoc_tx_ep->bEndpointAddress);

		usb_fill_int_urb(urb, data->udev, pipe,
					skb->data, skb->len,
					intel_btusb_isoc_tx_complete,
					skb, data->isoc_tx_ep->bInterval);

		urb->transfer_flags  = URB_ISO_ASAP;

		__intel_fill_isoc_descriptor(urb, skb->len,
				le16_to_cpu(data->isoc_tx_ep->wMaxPacketSize));

		hdev->stat.sco_tx++;
		goto skip_waking;

	default:
		return -EILSEQ;
	}

	err = intel_inc_tx(data);
	if (err) {
		usb_anchor_urb(urb, &data->deferred);
		schedule_work(&data->waker);
		err = 0;
		goto done;
	}

skip_waking:
	usb_anchor_urb(urb, &data->tx_anchor);

	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err < 0) {
		if (err != -EPERM && err != -ENODEV)
			BT_ERR("%s urb %p submission failed (%d)",
						hdev->name, urb, -err);
		kfree(urb->setup_packet);
		usb_unanchor_urb(urb);
	} else {
		usb_mark_last_busy(data->udev);
	}

done:
	usb_free_urb(urb);
	return err;
}

static void intel_btusb_notify(struct hci_dev *hdev, unsigned int evt)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);

	BT_DBG("%s evt %d", hdev->name, evt);

	if (hdev->conn_hash.sco_num != data->sco_num) {
		data->sco_num = hdev->conn_hash.sco_num;
		schedule_work(&data->work);
	}
}

static inline int __intel_set_isoc_interface(struct hci_dev *hdev,
						int altsetting)
{
	struct intel_btusb_data *data = hci_get_drvdata(hdev);
	struct usb_interface *intf = data->isoc;
	struct usb_endpoint_descriptor *ep_desc;
	int i, err;

	if (!data->isoc)
		return -ENODEV;

	err = usb_set_interface(data->udev, 1, altsetting);
	if (err < 0) {
		BT_ERR("%s setting interface failed (%d)", hdev->name, -err);
		return err;
	}

	data->isoc_altsetting = altsetting;

	data->isoc_tx_ep = NULL;
	data->isoc_rx_ep = NULL;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->isoc_tx_ep && usb_endpoint_is_isoc_out(ep_desc)) {
			data->isoc_tx_ep = ep_desc;
			continue;
		}

		if (!data->isoc_rx_ep && usb_endpoint_is_isoc_in(ep_desc)) {
			data->isoc_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->isoc_tx_ep || !data->isoc_rx_ep) {
		BT_ERR("%s invalid SCO descriptors", hdev->name);
		return -ENODEV;
	}

	return 0;
}

static void intel_btusb_work(struct work_struct *work)
{
	struct intel_btusb_data *data = container_of(work,
						struct intel_btusb_data, work);
	struct hci_dev *hdev = data->hdev;
	int new_alts;
	int err;

	if (hdev->conn_hash.sco_num > 0) {
		if (!test_bit(BTUSB_DID_ISO_RESUME, &data->flags)) {
			err = usb_autopm_get_interface(
					data->isoc ? data->isoc : data->intf);
			if (err < 0) {
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
				usb_kill_anchored_urbs(&data->isoc_anchor);
				return;
			}

			set_bit(BTUSB_DID_ISO_RESUME, &data->flags);
		}

		if (hdev->voice_setting & 0x0020) {
			static const int alts[3] = { 2, 4, 5 };
			new_alts = alts[hdev->conn_hash.sco_num - 1];
		} else {
			new_alts = hdev->conn_hash.sco_num;
		}

		if (data->isoc_altsetting != new_alts) {
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			usb_kill_anchored_urbs(&data->isoc_anchor);

			if (__intel_set_isoc_interface(hdev, new_alts) < 0)
				return;
		}

		if (!test_and_set_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
			if (intel_btusb_submit_isoc_urb(hdev, GFP_KERNEL) < 0)
				clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
			else
				intel_btusb_submit_isoc_urb(hdev, GFP_KERNEL);
		}
	} else {
		clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		usb_kill_anchored_urbs(&data->isoc_anchor);

		__intel_set_isoc_interface(hdev, 0);
		if (test_and_clear_bit(BTUSB_DID_ISO_RESUME, &data->flags))
			usb_autopm_put_interface(
					data->isoc ? data->isoc : data->intf);
	}
}

static void intel_btusb_waker(struct work_struct *work)
{
	struct intel_btusb_data *data = container_of(work,
					struct intel_btusb_data, waker);
	int err;

	err = usb_autopm_get_interface(data->intf);
	if (err < 0)
		return;

	usb_autopm_put_interface(data->intf);
}

static int intel_btusb_probe(struct usb_interface *intf,
				const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *ep_desc;
	struct intel_btusb_data *data;
	struct hci_dev *hdev;
	int i, err;

	BT_DBG("intf %p id %p", intf, id);

	/* interface numbers are hardcoded in the spec */
	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	data = devm_kzalloc(&intf->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (i = 0; i < intf->cur_altsetting->desc.bNumEndpoints; i++) {
		ep_desc = &intf->cur_altsetting->endpoint[i].desc;

		if (!data->intr_ep && usb_endpoint_is_int_in(ep_desc)) {
			data->intr_ep = ep_desc;
			continue;
		}

		if (!data->bulk_tx_ep && usb_endpoint_is_bulk_out(ep_desc)) {
			data->bulk_tx_ep = ep_desc;
			continue;
		}

		if (!data->bulk_rx_ep && usb_endpoint_is_bulk_in(ep_desc)) {
			data->bulk_rx_ep = ep_desc;
			continue;
		}
	}

	if (!data->intr_ep || !data->bulk_tx_ep || !data->bulk_rx_ep)
		return -ENODEV;

	data->cmdreq_type = USB_TYPE_CLASS;

	data->udev = interface_to_usbdev(intf);
	data->intf = intf;

	spin_lock_init(&data->lock);

	INIT_WORK(&data->work, intel_btusb_work);
	INIT_WORK(&data->waker, intel_btusb_waker);
	spin_lock_init(&data->txlock);

	init_usb_anchor(&data->tx_anchor);
	init_usb_anchor(&data->intr_anchor);
	init_usb_anchor(&data->bulk_anchor);
	init_usb_anchor(&data->isoc_anchor);
	init_usb_anchor(&data->deferred);

	hdev = hci_alloc_dev();
	if (!hdev)
		return -ENOMEM;

	hdev->bus = HCI_USB;
	hci_set_drvdata(hdev, data);

	data->hdev = hdev;

	SET_HCIDEV_DEV(hdev, &intf->dev);

	hdev->open     = intel_btusb_open;
	hdev->close    = intel_btusb_close;
	hdev->flush    = intel_btusb_flush;
	hdev->send     = intel_btusb_send_frame;
	hdev->notify   = intel_btusb_notify;

	/* Enable autosuspend for Intel Bluetooth Device */
	usb_enable_autosuspend(data->udev);

	/* Interface numbers are hardcoded in the specification */
	data->isoc = usb_ifnum_to_if(data->udev, 1);

	if (data->isoc) {
		err = usb_driver_claim_interface(&intel_btusb_driver,
							data->isoc, data);
		if (err < 0) {
			hci_free_dev(hdev);
			return err;
		}
	}

	data->patched = 0;

	err = hci_register_dev(hdev);
	if (err < 0) {
		hci_free_dev(hdev);
		return err;
	}

	usb_set_intfdata(intf, data);

	return 0;
}

static void intel_btusb_disconnect(struct usb_interface *intf)
{
	struct intel_btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev;

	BT_DBG("intf %p", intf);

	if (!data)
		return;

	hdev = data->hdev;
	usb_set_intfdata(data->intf, NULL);

	if (data->isoc)
		usb_set_intfdata(data->isoc, NULL);

	hci_unregister_dev(hdev);

	if (intf == data->isoc)
		usb_driver_release_interface(&intel_btusb_driver, data->intf);
	else if (data->isoc)
		usb_driver_release_interface(&intel_btusb_driver, data->isoc);

	hci_free_dev(hdev);
}

#ifdef CONFIG_PM
static int intel_btusb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct intel_btusb_data *data = usb_get_intfdata(intf);

	BT_DBG("intf %p", intf);

	if (data->suspend_count++)
		return 0;

	spin_lock_irq(&data->txlock);
	if (!(PMSG_IS_AUTO(message) && data->tx_in_flight)) {
		set_bit(BTUSB_SUSPENDING, &data->flags);
		spin_unlock_irq(&data->txlock);
	} else {
		spin_unlock_irq(&data->txlock);
		data->suspend_count--;
		return -EBUSY;
	}

	cancel_work_sync(&data->work);

	intel_btusb_stop_traffic(data);
	usb_kill_anchored_urbs(&data->tx_anchor);

	return 0;
}

static void play_deferred(struct intel_btusb_data *data)
{
	struct urb *urb;
	int err;

	while ((urb = usb_get_from_anchor(&data->deferred))) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err < 0)
			break;

		data->tx_in_flight++;
	}
	usb_scuttle_anchored_urbs(&data->deferred);
}

static int intel_btusb_resume(struct usb_interface *intf)
{
	struct intel_btusb_data *data = usb_get_intfdata(intf);
	struct hci_dev *hdev = data->hdev;
	int err = 0;

	BT_DBG("intf %p", intf);

	if (--data->suspend_count)
		return 0;

	if (!test_bit(HCI_RUNNING, &hdev->flags))
		goto done;

	if (test_bit(BTUSB_INTR_RUNNING, &data->flags)) {
		err = intel_btusb_submit_intr_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_INTR_RUNNING, &data->flags);
			goto failed;
		}
	}

	if (test_bit(BTUSB_BULK_RUNNING, &data->flags)) {
		err = intel_btusb_submit_bulk_urb(hdev, GFP_NOIO);
		if (err < 0) {
			clear_bit(BTUSB_BULK_RUNNING, &data->flags);
			goto failed;
		}

		intel_btusb_submit_bulk_urb(hdev, GFP_NOIO);
	}

	if (test_bit(BTUSB_ISOC_RUNNING, &data->flags)) {
		if (intel_btusb_submit_isoc_urb(hdev, GFP_NOIO) < 0)
			clear_bit(BTUSB_ISOC_RUNNING, &data->flags);
		else
			intel_btusb_submit_isoc_urb(hdev, GFP_NOIO);
	}

	spin_lock_irq(&data->txlock);
	play_deferred(data);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);
	schedule_work(&data->work);

	return 0;

failed:
	usb_scuttle_anchored_urbs(&data->deferred);
done:
	spin_lock_irq(&data->txlock);
	clear_bit(BTUSB_SUSPENDING, &data->flags);
	spin_unlock_irq(&data->txlock);

	return err;
}
#endif

static struct usb_driver intel_btusb_driver = {
	.name		= "intel_btusb",
	.probe		= intel_btusb_probe,
	.disconnect	= intel_btusb_disconnect,
#ifdef CONFIG_PM
	.suspend	= intel_btusb_suspend,
	.resume		= intel_btusb_resume,
#endif
	.id_table	= intel_btusb_table,
	.supports_autosuspend = 1,
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(intel_btusb_driver);

MODULE_AUTHOR("Marcel Holtmann <marcel@holtmann.org>");
MODULE_AUTHOR("Tedd Ho-Jeong An <tedd.an@intel.com>");
MODULE_DESCRIPTION("Intel Bluetooth USB driver for Chrome OS ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
