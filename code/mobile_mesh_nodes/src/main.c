/**
 * Partial implementation of the generic on off server and light HSL server models.
 * Not complete and therefore not compliant with the applicable specifications.
 * Provided for education purposes only.
 * 
 * Coded for and tested with Nordic Thingy
 * 
 **/

#include <stdlib.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <settings/settings.h>
#include <drivers/gpio.h>
#include <bluetooth/mesh.h>
#include <random/rand32.h>

#include "s4589514_pb.h"

#define BLE_MAX_NAME_LEN 	50
#define NUM_BEACONS 		13

// GPIO for the Thingy LED controller
const struct device *led_ctrlr;

static int nodeFamily = 0;
uint16_t hsl[8][3] = {
	{ 0x0000, 0x0000, 0x0000 }, // black
	{ 0x0000, 0xFFFF, 0x7FFF }, // red 
	{ 0x5555, 0xFFFF, 0x7FFF }, // green
	{ 0xAAAA, 0xFFFF, 0x7FFF }, // blue
	{ 0x2AAA, 0xFFFF, 0x7FFF }, // yellow
	{ 0xD555, 0xFFFF, 0x7FFF }, // magenta
	{ 0x7FFF, 0xFFFF, 0x7FFF }, // cyan
	{ 0x0000, 0x0000, 0xFFFF }  // white
};

struct NodeData
{
    char name[BLE_MAX_NAME_LEN];
    int8_t rssi;
};

static struct NodeData nodeData[NUM_BEACONS];

#define PORT "GPIO_P0"
#define LED_R 7
#define LED_G 5
#define LED_B 6

// states and state changes
uint8_t onoff_state;

uint16_t hsl_lightness;
uint16_t hsl_hue;
uint16_t hsl_saturation;
uint16_t rgb_r;
uint16_t rgb_g;
uint16_t rgb_b;

bool publish = false;
uint16_t reply_addr;
uint8_t reply_net_idx;
uint8_t reply_app_idx;

// device UUID
static uint8_t dev_uuid[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00 };

// Static node GAP names according to index
const char *nodeNames[] = {
    "4011-A",
    "4011-B",
    // "4011-C",
    "4011-D",
    "4011-E",
    "4011-F",
    "4011-G",
    "4011-H",
    "4011-I", // weak signal
    // "4011-J",
    // "4011-K",
    "4011-L", // weak signal
    "Relay_W",
    "Relay_X",
    "Relay_Y",
    "Relay_Z"
};

int8_t beaconRSSI[NUM_BEACONS][2] = {
	{ 0x00, 0x10 }, // iBeacon A
	{ 0x01, 0x20 }, // iBeacon B
	// { 0x0002, 0x0030 }, // iBeacon C
	{ 0x03, 0x40 }, // iBeacon D
	{ 0x04, 0x50 }, // iBeacon E
	{ 0x05, 0x60 }, // iBeacon F
	{ 0x06, 0x70 }, // iBeacon G
	{ 0x07, 0x80 }, // iBeacon H
	{ 0x08, 0x90 }, // iBeacon I
	{ 0x09, 0xA0 }, // iBeacon L
	{ 0x0A, 0x90 }, // Relay W
	{ 0x0B, 0xA0 }, // Relay X
	{ 0x0C, 0xB0 }, // Relay Y
	{ 0x0D, 0xC0 } // Relay Z
};

int8_t staticNodeRSSI[4][2];

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

void thingy_led_on(int r, int g, int b)
{
	// LEDs on Thingy are "active low" so zero means on. 
	r = !(r / 255);
	g = !(g / 255);
	b = !(b / 255);

	gpio_pin_set(led_ctrlr, LED_R, r);
	gpio_pin_set(led_ctrlr, LED_G, g);
	gpio_pin_set(led_ctrlr, LED_B, b);
}

void thingy_led_off()
{
	printk("GPIO_ACTIVE_LOW=%d\n",GPIO_ACTIVE_LOW);
	gpio_pin_set(led_ctrlr, LED_R, 1);
	gpio_pin_set(led_ctrlr, LED_G, 1);
	gpio_pin_set(led_ctrlr, LED_B, 1);
}

static void attention_on(struct bt_mesh_model *model)
{
	printk("attention_on()\n");
	thingy_led_on(255,0,0);
}

static void attention_off(struct bt_mesh_model *model)
{
	printk("attention_off()\n");
	thingy_led_off();
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static int provisioning_output_pin(bt_mesh_output_action_t action, uint32_t number) {
	printk("OOB Number: %u\n", number);
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

/*
 * The following two functions were converted from the pseudocode provided in the mesh models specification, section 6.1.1 Introduction
 */

double Hue_2_RGB(double v1, double v2, double vH ) {

	// printf("Hue_2_RGB: v1=%f v2=%f vH=%f\n",v1,v2,vH);

    if ( vH < 0.0f ) {
		vH += 1.0f;
	}
    if ( vH > 1.0f ) {
		vH -= 1.0f;
	}
    if (( 6.0f * vH ) < 1.0f ) {
		return ( v1 + ( v2 - v1 ) * 6.0f * vH );
	}
    if (( 2.0f * vH ) < 1.0f ) {
		return ( v2 );
	}
    if (( 3.0f * vH ) < 2.0f ) {
		return ( v1 + ( v2 - v1 ) * ( ( 2.0f / 3.0f ) - vH ) * 6.0f );
	}
    return ( v1 );
}	

void convert_hsl_to_rgb(unsigned short hsl_h,unsigned short hsl_s,unsigned short hsl_l ) {
	// printf("hsl_h=%d hsl_s=%d hsl_l=%d\n",hsl_h,hsl_s,hsl_l);
    double H = hsl_h / 65535.0f;
    double S = hsl_s / 65535.0f;
    double L = hsl_l / 65535.0f;
	double var_1 = 0.0f;
	double var_2 = 0.0f;
	
    if ( S == 0 ) {
      rgb_r = L * 255;
      rgb_g = L * 255;
      rgb_b = L * 255;
    } else {
      if ( L < 0.5f ) {
	      var_2 = L * ( 1.0f + S );
	  } else { 
		  var_2 = ( L + S ) - ( S * L );
	  }
      var_1 = 2.0f * L - var_2;
	  
      double R = Hue_2_RGB( var_1, var_2, H + ( 1.0f / 3.0f ));
      double G = Hue_2_RGB( var_1, var_2, H );
      double B = Hue_2_RGB( var_1, var_2, H - ( 1.0f / 3.0f ));
	  
	  // printf("R=%f G=%f B=%f\n",R,G,B);
	  
	  rgb_r = 256 * R;
	  rgb_g = 256 * G;
	  rgb_b = 256 * B;
    }
}

// message opcodes
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS BT_MESH_MODEL_OP_2(0x82, 0x04)

// need to forward declare as we have circular dependencies
void generic_onoff_status(bool publish, uint8_t on_or_off);

static void set_onoff_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf, bool ack)
{
	uint8_t msg_onoff_state = net_buf_simple_pull_u8(buf);
	if (msg_onoff_state == onoff_state) {
		// no state change so nothing to do
		return;
	}
	onoff_state = msg_onoff_state;
	uint8_t tid = net_buf_simple_pull_u8(buf);
	printk("set onoff state: onoff=%u TID=%u", onoff_state, tid);

	/*
	 * 3.7.7.2 Acknowledged Set
	 */ 
	if (ack) {
		generic_onoff_status(false, onoff_state);
	}

	/*
	 * If a server has a publish address, it is required to publish status on a state change
	 * See Mesh Profile Specification 3.7.6.1.2	
	 */

	if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
		generic_onoff_status(true, onoff_state);
	}

}

static void generic_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("gen_onoff_get\n");

	// logged for interest only
	printk("ctx net_idx=0x%02x\n",ctx->net_idx);
	printk("ctx app_idx=0x%02x\n",ctx->app_idx);
	printk("ctx addr=0x%02x\n",ctx->addr);
	printk("ctx recv_dst=0x%02x\n",ctx->recv_dst);
	reply_addr = ctx->addr;
	reply_net_idx = ctx->net_idx;
	reply_app_idx = ctx->app_idx;
	generic_onoff_status(false, onoff_state);
}

static void generic_onoff_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,	
			struct net_buf_simple *buf)
{
	printk("gen_onoff_set\n");
	set_onoff_state(model, ctx, buf, true);
}

static void generic_onoff_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, 
			struct net_buf_simple *buf)
{
	//check unicast address of static node - displayed on nrf mesh application
	if (ctx->addr == 5) {
		staticNodeRSSI[0][1] = ctx->recv_rssi;

	} else if (ctx->addr == 6) {
		staticNodeRSSI[1][1] = ctx->recv_rssi;

	} else if (ctx->addr == 7) {
		staticNodeRSSI[2][1] = ctx->recv_rssi;

	} else if (ctx->addr == 8) {
		staticNodeRSSI[3][1] = ctx->recv_rssi;

	}
}

static const struct bt_mesh_model_op generic_onoff_op[] = {
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_GET, 0, generic_onoff_get},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET, 2, generic_onoff_set},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK, 2, generic_onoff_set_unack},
		BT_MESH_MODEL_OP_END,
};

// model publication context
BT_MESH_MODEL_PUB_DEFINE(generic_onoff_pub, NULL, 2 + 1);


// Light HSL Server Model - minimal subset only - would not be deemed compliant
// -------------------------------------------------------------------------------------------------------

// message opcodes

#define BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x77)

// NB: only unacknowledged light_hsl_set is implemented in this code
static void set_hsl_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, 
			struct net_buf_simple *buf) {

    uint16_t msg_hsl_lightness = net_buf_simple_pull_le16(buf);
    uint16_t msg_hsl_hue = net_buf_simple_pull_le16(buf);
    uint16_t msg_hsl_saturation = net_buf_simple_pull_le16(buf);
    if (msg_hsl_lightness == hsl_lightness && msg_hsl_hue == hsl_hue && msg_hsl_saturation == hsl_saturation) {
        // no state change so nothing to do
        return;
    }
    hsl_lightness = msg_hsl_lightness;
    hsl_hue = msg_hsl_hue;
    hsl_saturation = msg_hsl_saturation;
    printk("set HSL state: lightness=%u hue=%u saturation=%u\n", hsl_lightness, hsl_hue, hsl_saturation);
    convert_hsl_to_rgb(hsl_hue,hsl_saturation,hsl_lightness);
    if (onoff_state == 1) {
        thingy_led_on(rgb_r, rgb_g, rgb_b);
    }
    /*
    * If a server has a publish address, it is required to publish status on a state change
    * See Mesh Profile Specification 3.7.6.1.2
    */
    if (model->pub->addr != BT_MESH_ADDR_UNASSIGNED) {
        // if we had implemented light HSL status messages, we'd send one here
        printk("A status message should be sent here - not implemented\n");
    }
}

static void light_hsl_set_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx, 
			struct net_buf_simple *buf) {
    printk("light_hsl_set_unack\n");
	int8_t temp = net_buf_simple_pull_u8(buf);
	
    // set_hsl_state(model, ctx, buf);
}

// static const struct bt_mesh_model_op light_hsl_op[] = {
// 		{BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK, 1, light_hsl_set_unack},
// 		BT_MESH_MODEL_OP_END,
// };

// model publication context
BT_MESH_MODEL_PUB_DEFINE(light_hsl_pub, NULL, 2 + 6);

// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

static void get_proximity_data(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {

	int8_t proximity_alert = net_buf_simple_pull_u8(buf);
	int8_t random = net_buf_simple_pull_u8(buf);
	printk("prox %d\n", proximity_alert);
	printk("rand %d\n", random);

	if (proximity_alert == 1) {
		
		// do something
	} else if (proximity_alert == 1) {
		// do something else
	}
	
}

static void get_proximity_data_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf) {

	get_proximity_data(model, ctx, buf);
}

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

#define BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK		BT_MESH_MODEL_OP_2(0x82, 0x40) // MOBILE A
#define BT_MESH_MODEL_OP_MOBILE_2_TO_BASE_UNACK 	BT_MESH_MODEL_OP_2(0x82, 0x41) // MOBILE B

// 12 bytes (5 sets of beaconRSSI data (2 bytes each), init msg, tid value)
BT_MESH_MODEL_PUB_DEFINE(data_mobile_to_base, NULL, 13);
BT_MESH_MODEL_PUB_DEFINE(data_mobile_to_base_2, NULL, 13); 

// mobile 1 receiving data from base (proximity)
static const struct bt_mesh_model_op prox_data_from_base[] = {
	{BT_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK, 2, get_proximity_data_unack},
	BT_MESH_MODEL_OP_END,
};

// mobile 1 receiving data from base (proximity)
// static const struct bt_mesh_model_op prox_data_from_base_2[] = {
// 	{BT_MESH_MODEL_OP_MOBILE_2_TO_BASE_UNACK, 1, get_proximity_data_unack},
// 	BT_MESH_MODEL_OP_END,
// };


static struct bt_mesh_model sig_models[] = {
    BT_MESH_MODEL_CFG_SRV,
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, generic_onoff_op, &generic_onoff_pub, NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_HSL_SRV, prox_data_from_base, &light_hsl_pub, NULL),
	// BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_XYL_CLI, NULL, &data_mobile_to_base, &beaconRSSI[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_XYL_CLI, NULL, &data_mobile_to_base, &beaconRSSI[0]),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_LIGHT_LC_CLI, NULL, &data_mobile_to_base_2, &beaconRSSI[0]),
};

// node contains elements.note that BT_MESH_MODEL_NONE means "none of this type" ands here means "no vendor models"
static struct bt_mesh_elem elements[] = {
		BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
		.cid = 0xFFFF,
		.elem = elements,
		.elem_count = ARRAY_SIZE(elements),
};
// Mobile to Base data model 

static uint8_t msg_tid = 0; 
int strongestSignals[13];

int sendDataToBase(uint16_t message_type) {
	int err;
	struct bt_mesh_model* model;
	if (message_type == BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK) {
		model = &sig_models[4];
	} else if (message_type == BT_MESH_MODEL_OP_MOBILE_2_TO_BASE_UNACK) {
		model = &sig_models[5];
	}
	
	printk("send data to base...");
	if (model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the light HSL client model - \
					add one with a configuration app like nRF Mesh\n");
		return -1;
	}

	int numMsgsAdded = 0;

	struct net_buf_simple* msg = model->pub->msg;
	bt_mesh_model_msg_init(msg, message_type);
	for (int i = 0; i < NUM_BEACONS; i++) {
		if (strongestSignals[i] == 1) {
			net_buf_simple_add_u8(msg, beaconRSSI[i][1]); 
			net_buf_simple_add_u8(msg, beaconRSSI[i][0]);
			numMsgsAdded++;
			// only send a set of 5 rssi/name values;
			if (numMsgsAdded == 4) {
				break;
			}
		}
	}
	// add empty data sets to fill transmit buffer as
	// base node is expecting a certain amount of data
	while (numMsgsAdded < 5) {
		net_buf_simple_add_u8(msg, 0xFF); 
		net_buf_simple_add_u8(msg, 0x0E); //TODO change
		numMsgsAdded++;
	}
	net_buf_simple_add_u8(msg, nodeFamily);
	net_buf_simple_add_u8(msg, msg_tid);
	msg_tid += 2;
	printk("publishing RSSI data\n");
	err = bt_mesh_model_publish(model);
	if (err) {
		printk("bt_mesh_model_publish err %d\n", err);
	}
	return err;
}



// ----------------------------------------------------------------------------------------------------
// generic onoff status TX message producer

// Either publish a status message to the publish address associated with the generic on off server model
// or send it to the specified address
void generic_onoff_status(bool publish, uint8_t on_or_off)
{
    int err;
    struct bt_mesh_model *model = &sig_models[2];
	if (publish && model->pub->addr == BT_MESH_ADDR_UNASSIGNED) {
		printk("No publish address associated with the generic on off server model - \
					add one with a configuration app like nRF Mesh\n");
		return;
	} 

	if (publish) {
	    struct net_buf_simple *msg = model->pub->msg;
		net_buf_simple_reset(msg);
		bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
		net_buf_simple_add_u8(msg, on_or_off);
		printk("publishing on off status message\n");		
		err = bt_mesh_model_publish(model);
		if (err) {
			printk("bt_mesh_model_publish err %d\n", err);
		}
	} else {
		uint8_t buflen = 7;
		NET_BUF_SIMPLE_DEFINE(msg, buflen);
		bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
		net_buf_simple_add_u8(&msg, on_or_off);
		struct bt_mesh_msg_ctx ctx = {
				.net_idx = reply_net_idx,
				.app_idx = reply_app_idx,
				.addr = reply_addr,
				.send_ttl = BT_MESH_TTL_DEFAULT,
		};

		printk("sending on off status message\n");
		if (bt_mesh_model_send(model, &ctx, &msg, NULL, NULL)) {
			printk("Unable to send generic onoff status message\n");
		}
	}
}

static void bt_ready(int err)
{
	if (err)
	{
		printk("bt_enable init failed with err %d", err);
		return;
	}
    printk("Bluetooth initialised OK\n");

	gen_uuid();

    printk("\n%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n\n",
            dev_uuid[15], dev_uuid[14], dev_uuid[13], dev_uuid[12],dev_uuid[11], dev_uuid[10], dev_uuid[9], dev_uuid[8],
            dev_uuid[7], dev_uuid[6], dev_uuid[5], dev_uuid[4],dev_uuid[3], dev_uuid[2], dev_uuid[1], dev_uuid[0]);

	err = bt_mesh_init(&prov, &comp);

	if (err)
	{
		printk("bt_mesh_init failed with err %d", err);
		return;
	}

	printk("Mesh initialised OK: 0x%04x\n",elements[0].addr);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	    printk("Settings loaded: 0x%04x\n",elements[0].addr);
	}

    if (!bt_mesh_is_provisioned()) {
    	printk("Node has not been provisioned - beaconing\n");
		bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);
	} else {
		
    	printk("Node has already been provisioned\n");
	    printk("Node unicast address: 0x%04x\n",elements[0].addr);
	}

}

static void configure_thingy_led_controller()
{
	led_ctrlr = device_get_binding(PORT);
	gpio_pin_configure(led_ctrlr, LED_R, GPIO_OUTPUT);
	gpio_pin_configure(led_ctrlr, LED_G, GPIO_OUTPUT);
	gpio_pin_configure(led_ctrlr, LED_B, GPIO_OUTPUT);
}

void indicate_on() {
	int r = 0, g = 0, b = 255;
	thingy_led_on(r, g, b);
    k_sleep(K_MSEC(5000));
	thingy_led_off();	
}

/**
 * @brief Callback function to extract GAP name from BLE
 *          advertisements
 * 
 * @param data : Data to parse
 * @param user_data : Container to save parsed data to
 * @return true : If parsing should continue
 * @return false : If parsing should stop
 */
static bool parse_advertising_data(struct bt_data *data, void *user_data)
{
    struct NodeData *parsedData = user_data;

    // Check if the data is the advertiser's name
    if (data->type == BT_DATA_NAME_COMPLETE)
    {

        // If the name length is valid, save it and stop parsing
        if (data->data_len < BLE_MAX_NAME_LEN)
        {

            memcpy(parsedData->name, data->data, data->data_len);
            parsedData->name[data->data_len] = '\0';

            return false;
        }
    }

    return true;
}

static int8_t beaconCount = 0;

/**
 * @brief Callback for when an advertisement has been found
 * 
 * @param addr : BLE address of the advertiser
 * @param rssi : Received signal strength indicator of the advertisement
 * @param advType : Type of advertisement
 * @param ad : The scanned advertisement
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t advType, struct net_buf_simple *ad)
{
    struct NodeData adData = {0};
 
	bt_data_parse(ad, parse_advertising_data, &adData);

    for (int i = 0; i < NUM_BEACONS; i++)
    {
        // See if a valid node; if so, save data
        if (strcmp(adData.name, nodeNames[i]) == 0) {
			beaconRSSI[i][1] = rssi;		
        }
    }
}

static void check_family_colour() {
	int tempFam = get_node_family();

	if (nodeFamily != tempFam) {
		nodeFamily = tempFam;
		convert_hsl_to_rgb(hsl[nodeFamily][0], hsl[nodeFamily][1], hsl[nodeFamily][2]);
		thingy_led_on(rgb_r, rgb_g, rgb_b);
	}
}

static void reset_beacon_values() {
	for (int i = 0; i < NUM_BEACONS; i++) {
		beaconRSSI[i][1] = -127;
		nodeData[i].rssi = -127;
		strongestSignals[i] = 0;
	}
	beaconCount = 0; // reset count index
}

void main(void)
{
	printk("thingy light node v1.1.0\n");

	configure_thingy_led_controller();
    indicate_on();

	init_pb();

	// set default colour to white
	rgb_r = 255;
	rgb_g = 255;
	rgb_b = 255;

	printk("Calling bt_enable");
	int err = bt_enable(bt_ready);
	if (err)
	{
		printk("bt_enable failed with err %d", err);
	}

	// tid's are even for mobile A and odd for mobile B
	if (strcmp(bt_get_name(), "Mobile_A") == 0) {
		msg_tid = 0;
	} else if (strcmp(bt_get_name(), "Mobile_B") == 0) {
		msg_tid = 1;
	}

	struct bt_le_scan_param scanParams = {
		.type = BT_HCI_LE_SCAN_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = 0x0200,
		.window = 0x0200
	};

	while (1) {
		// read RSSI data
		// k_msleep(500);
		check_family_colour();

		if (bt_mesh_is_provisioned()) {
			
			bt_mesh_suspend();
			reset_beacon_values();
			k_msleep(100);
			
			err = bt_le_scan_start(&scanParams, scan_cb);
			if (err) {
				printk("starting scanning failed (err %d)\n", err);
			}

			k_msleep(300);
			err = bt_le_scan_stop();
			if (err) {
				printk("stop scanning failed (err %d)\n", err);
			}
		
			bt_mesh_resume();
			k_msleep(200);

			for (int i = 0; i < 4; i++) {
				beaconRSSI[i + 9][1] = staticNodeRSSI[i][1];
				// printk("i: %d    rssi: %d\n", i, beaconRSSI[i][1]);
			}
			if (beaconRSSI[2][1] > -110) {
				beaconRSSI[2][1] = beaconRSSI[2][1] - 7;
				if (beaconRSSI[2][1] > -50) {
					beaconRSSI[2][1] = beaconRSSI[2][1] + 7;
				}
			}


			for (int j = 0; j < NUM_BEACONS; j++) {
				int count = 0;
				printk("%s: %d\n", nodeNames[j], beaconRSSI[j][1]);
				for (int k = 0; k < NUM_BEACONS; k++) {
					
					if (beaconRSSI[j][1] > beaconRSSI[k][1]) {
						count++;
						if (count >= 9) { // 8 for top 5 beacons --> 9 for top 4 beacons
							strongestSignals[j] = 1;
							break;
						}
					}
				}
			}

			for (int i = 0; i < 4; i++) {
				staticNodeRSSI[i][1] = -127;
			}
			// k_msleep(500);
			if (strcmp(bt_get_name(), "Mobile_A") == 0) {
				if (sendDataToBase(BT_MESH_MODEL_OP_MOBILE_TO_BASE_UNACK)) {
					printk("unable to send RSSI data\n");
				}
			} else if (strcmp(bt_get_name(), "Mobile_B") == 0) {
				if (sendDataToBase(BT_MESH_MODEL_OP_MOBILE_2_TO_BASE_UNACK)) {
					printk("unable to send RSSI data\n");
				}
			}
			
			k_msleep(1500);
			
		} else {
			k_msleep(500);
		}
	}
}
