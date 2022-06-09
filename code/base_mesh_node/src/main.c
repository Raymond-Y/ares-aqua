/**
 * On/Off Switch Node - behaviour predominantly based on the Bluetooth mesh Generic OnOff Client Model
 * 
 * Coded for and tested with nRF52840 DK board
 * 
 **/

#include <drivers/gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <settings/settings.h>
#include <bluetooth/mesh/proxy.h>
#include <random/rand32.h>

#include <device.h>
#include <devicetree.h>
#include <data/json.h>

#include <toolchain.h>
#include <stdarg.h>
#include <inttypes.h>

#include <init.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <usb/usb_device.h>

#include <sys/byteorder.h>
#include <sys/util.h>

// GPIO for the buttons - see zephyr samples/button for origin of some of this code
// Get button configuration from the devicetree sw0 alias. This is mandatory.

// Button 1
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
							      {0});
static struct gpio_callback gpio_btn1_cb;

#ifndef SW0_GPIO_FLAGS
#ifdef SW0_GPIO_PIN_PUD
#define SW0_GPIO_FLAGS SW0_GPIO_PIN_PUD
#else
#define SW0_GPIO_FLAGS 0
#endif
#endif

#define BUTTON_DEBOUNCE_DELAY_MS 250

static uint32_t btn_time[4] = { 0,0,0,0};
static uint32_t btn_last_time[4] = { 0,0,0,0};

static struct gpio_callback gpio_btn1_cb;

static struct k_sem recvMobileMsg;

// GPIO for LED 0
/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});

// for use with k_work_submit which we use to handle button presses in a background thread to avoid holding onto an IRQ for too long
static struct k_work button1_work;


static uint8_t dev_uuid[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 };

static uint8_t onoff_tid;
static uint8_t hsl_tid;

void gen_uuid() {
    uint32_t rnd1 = sys_rand32_get();
    uint32_t rnd2 = sys_rand32_get();
    uint32_t rnd3 = sys_rand32_get();
    uint32_t rnd4 = sys_rand32_get();

    dev_uuid[15] = (rnd1 >> 24) & 0x0FF;
    dev_uuid[14] = (rnd1 >> 16) & 0x0FF;
    dev_uuid[13] = (rnd1 >>  8) & 0x0FF;
    dev_uuid[12] =  rnd1 & 0x0FF;

    dev_uuid[11] = (rnd2 >> 24) & 0x0FF;
    dev_uuid[10] = (rnd2 >> 16) & 0x0FF;
    dev_uuid[9] = (rnd2 >>  8) & 0x0FF;
    dev_uuid[8] =  rnd2 & 0x0FF;

    dev_uuid[7] = (rnd3 >> 24) & 0x0FF;
    dev_uuid[6] = (rnd3 >> 16) & 0x0FF;
    dev_uuid[5] = (rnd3 >>  8) & 0x0FF;
    dev_uuid[4] =  rnd3 & 0x0FF;

    dev_uuid[3] = (rnd4 >> 24) & 0x0FF;
    dev_uuid[2] = (rnd4 >> 16) & 0x0FF;
    dev_uuid[1] = (rnd4 >>  8) & 0x0FF;
    dev_uuid[0] =  rnd4 & 0x0FF;

    /* Set 4 MSB bits of time_hi_and_version field */
    dev_uuid[6] &= 0x0f;
    dev_uuid[6] |= 4 << 4;

    /* Set 2 MSB of clock_seq_hi_and_reserved to 10 */
    dev_uuid[8] &= 0x3f;
    dev_uuid[8] |= 0x80;

}

void ledOn(void) {
	gpio_pin_set(led.port, led.pin, 1);
}

void ledOff(void) {
	gpio_pin_set(led.port, led.pin, 0);
}

static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
	ledOn();
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
	ledOff();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action, uint32_t number) {
	printk("OOB Number: %04d\n", number);
	return 0;
}

static void provisioning_complete(uint16_t net_idx, uint16_t addr) {
    printk("Provisioning completed\n");
}

static void provisioning_reset(void)
{
	bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
}

// provisioning properties and capabilities
static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
	.output_size = 4,
	.output_actions = BT_MESH_DISPLAY_NUMBER,
	.output_number = provisioning_output_pin,
	.complete = provisioning_complete,
	.reset = provisioning_reset,
};

// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

// -------------------------------------------------------------------------------------------------------
// Generic OnOff Client Model
// --------------------------

uint8_t onoff[] = {
	0,
	1};

// handler functions for this model's RX messages

static void generic_onoff_status(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
	uint8_t onoff_state = net_buf_simple_pull_u8(buf);
	printk("generic_onoff_status onoff=%d\n",onoff_state);   
}

// generic on off client - message types defined by this model

#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET	      BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET	      BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK  BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS     BT_MESH_MODEL_OP_2(0x82, 0x04)

// BEACON data receive (MOBILE TO BASE)
#define BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x40) // Mobile_A

#define BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK_2	BT_MESH_MODEL_OP_2(0x82, 0x41) // Mobile_B


static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS, 1, generic_onoff_status},
		BT_MESH_MODEL_OP_END,
};

static void analyse_received_data(int8_t* msg_rssi_value, int8_t* msg_rssi_node) {
	for (int i = 0; i < 4; i++) {
		if (msg_rssi_node[i] == 0x00) {
			printk("0:%d,", msg_rssi_value[i]); // 4011-A
		} else if (msg_rssi_node[i] == 0x01) {
			printk("1:%d,", msg_rssi_value[i]); // 4011-B
		} else if (msg_rssi_node[i] == 0x03) {
			printk("2:%d,", msg_rssi_value[i]); // 4011-D
		} else if (msg_rssi_node[i] == 0x04) {
			printk("3:%d,", msg_rssi_value[i]); // 4011-E
		} else if (msg_rssi_node[i] == 0x05) {
			printk("4:%d,", msg_rssi_value[i]); // 4011-F
		} else if (msg_rssi_node[i] == 0x06) {
			printk("5:%d,", msg_rssi_value[i]); // 4011-G
		} else if (msg_rssi_node[i] == 0x07) {
			printk("6:%d,", msg_rssi_value[i]); // 4011-H
		} else if (msg_rssi_node[i] == 0x0A) {
			printk("7:%d,", msg_rssi_value[i]); // Relay-W
		} else if (msg_rssi_node[i] == 0x0B) {
			printk("8:%d,", msg_rssi_value[i]); // Relay-X
		} else if (msg_rssi_node[i] == 0x0C) {
			printk("9:%d,", msg_rssi_value[i]); // Relay-Y
		} else if (msg_rssi_node[i] == 0x0D) {
			printk("10:%d,", msg_rssi_value[i]); // Relay-Z
		} 
	}
	printk("}\n");
}

static void get_data_from_mobile(struct bt_mesh_model* model, struct bt_mesh_msg_ctx* ctx, struct net_buf_simple* buf) {

	int8_t msg_rssi_value[5];
	int8_t msg_rssi_node[5];
	int8_t msg_family;

	for (int i = 0; i < 5; i++) {
		msg_rssi_value[i] = net_buf_simple_pull_u8(buf);
		msg_rssi_node[i] = net_buf_simple_pull_u8(buf);
	}
	msg_family = net_buf_simple_pull_u8(buf);

	if (ctx->addr == 4) {
		//mobile node 1
		printk("{%dMobile1, ", msg_family);
	} else if (ctx->addr == 9) {
		// mobile node 2
		printk("{%dMobile2, ", msg_family);
	}
	
	// printk("rssi YAY: %d\n", ctx->recv_rssi);
	analyse_received_data(msg_rssi_value, msg_rssi_node);
	
	// printk("\n node: %d: %d\n", msg_rssi_node, msg_rssi_value);
	if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        // if we had implemented light HSL status messages, we'd send one here
        // printk("A status message should be sent here - not implemented\n");
    }
}

static void get_data_from_mobile_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {

	k_sem_take(&recvMobileMsg, K_FOREVER);
	get_data_from_mobile(model, ctx, buf);
	k_sem_give(&recvMobileMsg);

}


static const struct bt_mesh_model_op rssi_data_from_mobile_op[] = {
	{BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK, 2 + 2*5, get_data_from_mobile_unack},
	BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model_op rssi_data_from_mobile_op_2[] = {
	{BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK_2, 2 + 2*5, get_data_from_mobile_unack},
	BT_MESH_MODEL_OP_END,
};

BT_MESH_MODEL_PUB_DEFINE(rssi_data_from_mobile_pub, NULL, 13);
BT_MESH_MODEL_PUB_DEFINE(rssi_data_from_mobile_pub_2, NULL, 13);

#define NUMBER_OF_COLOURS 8

uint16_t hsl[NUMBER_OF_COLOURS][3] = {
	{ 0x0000, 0x0000, 0x0000 }, // black
	{ 0x0000, 0xFFFF, 0x7FFF }, // red 
	{ 0x5555, 0xFFFF, 0x7FFF }, // green
	{ 0xAAAA, 0xFFFF, 0x7FFF }, // blue
	{ 0x2AAA, 0xFFFF, 0x7FFF }, // yellow
	{ 0xD555, 0xFFFF, 0x7FFF }, // magenta
	{ 0x7FFF, 0xFFFF, 0x7FFF }, // cyan
	{ 0x0000, 0x0000, 0xFFFF }  // white
};

uint8_t current_hsl_inx = 1;

// message types defined by this model.
#define BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x77)

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

BT_MESH_MODEL_PUB_DEFINE(gen_onoff_cli, NULL, 2);
BT_MESH_MODEL_PUB_DEFINE(light_hsl_cli, NULL, 2);

int8_t temp = 0x00;

static struct bt_mesh_model sig_models[] = {
				BT_MESH_MODEL_CFG_SRV,
				BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
				BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op, &gen_onoff_cli, &onoff[0]),
				BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_HSL_CLI, NULL, &light_hsl_cli, &temp),
				BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_XYL_SRV, rssi_data_from_mobile_op, &rssi_data_from_mobile_pub, &temp),
				BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_LC_SRV, rssi_data_from_mobile_op_2, &rssi_data_from_mobile_pub_2, &temp),
};

// node contains elements. Note that BT_MESH_MODEL_NONE means "none of this type" and here means "no vendor models"
static struct bt_mesh_elem elements[] = {
				BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
				.cid = 0xFFFF,
				.elem = elements,
				.elem_count = ARRAY_SIZE(elements),
};


// Generic OnOff Client - TX message producer functions
// -----------------------------------------------------------

static int msg_tid = 0; // TODO: move

int genericOnOffGet() 
{
	printk("genericOnOffGet\n");
    int err;
    struct bt_mesh_model *model = &sig_models[2];
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the generic on off client model - add one with a configuration app like nRF Mesh\n");
		return -1;
	}     
	struct net_buf_simple *msg = model->pub->msg;

	bt_mesh_model_msg_init(msg,	BT_MESH_MODEL_OP_GENERIC_ONOFF_GET);
	printk("publishing get on off message\n");
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}

	return err;
}

int sendGenOnOffSet(uint8_t on_or_off, uint16_t message_type) 
{
    int err;
    struct bt_mesh_model *model = &sig_models[2];
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the generic on off client model - add one with a configuration app like nRF Mesh\n");
		return -1;
	}     
	struct net_buf_simple *msg = model->pub->msg;

	bt_mesh_model_msg_init(msg,	message_type);
	net_buf_simple_add_u8(msg, on_or_off);
	net_buf_simple_add_u8(msg, onoff_tid);
	onoff_tid++;
	printk("publishing set on off state=0x%02x\n",on_or_off);
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}

	return err;
}

void genericOnOffSetUnAck(uint8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK))
	{
		printk("Unable to send generic onoff set unack message\n");
	} else {
	    printk("onoff set unack message %d sent\n",on_or_off);
	}
}


void genericOnOffSet(uint8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET))
	{
		printk("Unable to send generic onoff set message\n");
	} else {
		printk("onoff set message %d sent\n",on_or_off);
	}
}


// Light HSL Client - TX message producer functions
// -----------------------------------------------------------

int sendLightHslSet(uint16_t message_type) 
{
    int err;
	struct bt_mesh_model* model = &sig_models[3];
	// printk("sending proximity data to mobile..\n");
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the light HSL client model - \
					add one with a configuration app like nRF Mesh\n");
		return -1;
	}

	struct net_buf_simple* msg = model->pub->msg;
	bt_mesh_model_msg_init(msg, message_type);
	net_buf_simple_add_u8(msg, 0x01);
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
	msg_tid++;

	return err;	
}

void lightHslSetUnAck() 
{
	if (sendLightHslSet(BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK))
	{
		printk("Unable to send light HSL set unack message\n");
	}
}

// Buttons
// -------
void button1_work_handler(struct k_work *work)
{	
	// bt_mesh_reset();
    lightHslSetUnAck();
}

bool debounce(int btn_inx) {
	bool ignore = false;
	btn_time[btn_inx] = k_uptime_get_32();
	if (btn_time[btn_inx] < (btn_last_time[btn_inx] + BUTTON_DEBOUNCE_DELAY_MS)) {
		ignore = true;
	} else {
		ignore = false;
	}
	btn_last_time[btn_inx] = btn_time[btn_inx];
	return ignore;
}

void button_1_pressed(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	printk("Button 1 pressed at %d\n", k_cycle_get_32());

    if (!debounce(0)) {
	  k_work_submit(&button1_work);
	}
}

// -------------------------------------------------------------------------------------------------------
// LED
// -------

void configureLED(void)
{
	printk("configureLED\n");

	int ret = 0;

	if (led.port && !device_is_ready(led.port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}
	if (led.port) {
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n",
			       ret, led.port->name, led.pin);
			led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}

	// LED 0
	ledOff();
}


// -------------------------------------------------------------------------------------------------------
// Buttons
// -------

void configureButtons(void) {
	printk("configureButtons\n");

	int ret;

	// Button 1
	k_work_init(&button1_work, button1_work_handler);
	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, button1.port->name, button1.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button1,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			ret, button1.port->name, button1.pin);
		return;
	}

	gpio_init_callback(&gpio_btn1_cb, button_1_pressed, BIT(button1.pin));
	gpio_add_callback(button1.port, &gpio_btn1_cb);
	printk("Set up button at %s pin %d\n", button1.port->name, button1.pin);

}


// static const struct bt_data ad[] = {
//     BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
//                             0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69, 0x69),

// };

static void bt_ready(int err)
{
	if (err)
	{
			printk("bt_enable init failed with err %d\n", err);
			return;
	}

	printk("Bluetooth initialised OK\n");

	// prov is a bt_mesh_prov struct and is declared in provisioning.c
	err = bt_mesh_init(&prov, &comp);
	if (err)
	{
			printk("bt_mesh_init failed with err %d\n", err);
			return;
	}

	printk("Mesh initialised OK\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	    printk("Settings loaded\n");
	}

	// err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	// if (err) {
	// 	printk("\nAdvertising failed to start (err %d)\n", err);
	// 	return;
	// }

    if (!bt_mesh_is_provisioned()) {
    	printk("Node has not been provisioned - beaconing\n");
		gen_uuid();
		printk("\n%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n\n",
				dev_uuid[15], dev_uuid[14], dev_uuid[13], dev_uuid[12],dev_uuid[11], dev_uuid[10], dev_uuid[9], dev_uuid[8],
				dev_uuid[7], dev_uuid[6], dev_uuid[5], dev_uuid[4],dev_uuid[3], dev_uuid[2], dev_uuid[1], dev_uuid[0]);
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	} else {
    	printk("Node has already been provisioned\n");
	    printk("Node unicast address: 0x%04x\n",elements[0].addr);
	}

}


// uint8_t uart_buf[16];
// static void uart_cb(const struct device *dev, void *user_data) {

// 	// uart_irq_update(x);
// 	int data_length = 0;
// 	memset(uart_buf, 0, sizeof(uart_buf));

// 	if (uart_irq_rx_ready(dev)) {
// 		data_length = uart_fifo_read(dev, uart_buf, sizeof(uart_buf));
// 		uart_buf[data_length] = 0;
// 		if (sendProximityDataToMobile(BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK_2)) {
// 			printk("unable to send RSSI data\n");
// 		}
// 		// if (sendProximityDataToMobile(BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK)) {
// 		// 	printk("unable to send RSSI data\n");
// 		// }
// 		// printk("%s\n", uart_buf);
// 	}
	
// }


void main(void)
{
	int err;
	// const struct device* dev = device_get_binding("CDC_ACM_0");
	// if (!device_is_ready(dev)) {
	// 	return;
	// }	
	

	if (usb_enable(NULL)) {
		return;
	}
	
	// uart_irq_callback_set(dev, uart_cb);

	/* Enable rx interrupts */
	// uart_irq_rx_enable(dev);
	
	onoff_tid = 0;
	hsl_tid = 0;

	k_sem_init(&recvMobileMsg, 0, 1);
	k_sem_give(&recvMobileMsg);	

	configureButtons();
	configureLED();


	// const struct device* uart = device_get_binding("UART_0");
	// int uart1_data = 3;
	// uart_irq_callback_user_data_set(uart, uart_cb, &uart1_data);
	// uart_irq_rx_enable(uart);
	// printk("uart enabled\n");
	
	err = bt_enable(bt_ready);
	if (err) {
			printk("bt_enable failed with err %d\n", err);
	}



	while (1) {
		k_msleep(2000);
		// if (sendProximityDataToMobile(BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK_2)) {
		// 	printk("unable to send RSSI data\n");
		// }
		if (sendLightHslSet(BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK)) {
			printk("unable to send RSSI data\n");
		}

	}
}
