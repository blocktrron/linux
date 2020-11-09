// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2019 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
/*
 *                   +----------------------+
 * GMAC1----RGMII----|--MAC0                |
 *      \---MDIO1----|--REGs                |----MDIO3----\
 *                   |                      |             |  +------+
 *                   |                      |             +--|      |
 *                   |                 MAC1-|----RMII--M-----| PHY0 |-o P0
 *                   |                      |          |  |  +------+
 *                   |                      |          |  +--|      |
 *                   |                 MAC2-|----RMII--------| PHY1 |-o P1
 *                   |                      |          |  |  +------+
 *                   |                      |          |  +--|      |
 *                   |                 MAC3-|----RMII--------| PHY2 |-o P2
 *                   |                      |          |  |  +------+
 *                   |                      |          |  +--|      |
 *                   |                 MAC4-|----RMII--------| PHY3 |-o P3
 *                   |                      |          |  |  +------+
 *                   |                      |          |  +--|      |
 *                   |                 MAC5-|--+-RMII--M-----|-PHY4-|-o P4
 *                   |                      |  |       |     +------+
 *                   +----------------------+  |       \--CFG_SW_PHY_SWAP
 * GMAC0---------------RMII--------------------/        \-CFG_SW_PHY_ADDR_SWAP
 *      \---MDIO0--NC
 *
 * GMAC0 and MAC5 are connected together and use same PHY. Depending on
 * configuration it can be PHY4 (default) or PHY0. Only GMAC0 or MAC5 can be
 * used at same time. If GMAC0 is used (default) then MAC5 should be disabled.
 *
 * CFG_SW_PHY_SWAP - swap connections of PHY0 and PHY4. If this bit is not set
 * PHY4 is connected to GMAC0/MAC5 bundle and PHY0 is connected to MAC1. If this
 * bit is set, PHY4 is connected to MAC1 and PHY0 is connected to GMAC0/MAC5
 * bundle.
 *
 * CFG_SW_PHY_ADDR_SWAP - swap addresses of PHY0 and PHY4
 *
 * CFG_SW_PHY_SWAP and CFG_SW_PHY_ADDR_SWAP are part of SoC specific register
 * set and not related to switch internal registers.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <net/dsa.h>

#define AR9331_SW_NAME				"ar9331_switch"
#define AR9331_SW_PORTS				6

#define AR9331_CPU_PORT				0

/* dummy reg to change page */
#define AR9331_SW_REG_PAGE			0x40000

/* Global Interrupt */
#define AR9331_SW_REG_GINT			0x10
#define AR9331_SW_REG_GINT_MASK			0x14
#define AR9331_SW_GINT_PHY_INT			BIT(2)

#define AR9331_SW_REG_FLOOD_MASK		0x2c
#define AR9331_SW_FLOOD_MASK_BROAD_TO_CPU	BIT(26)
#define AR9344_SW_FLOOD_MASK_BC_DP_M		GENMASK(31, 25)
#define AR9344_SW_FLOOD_MASK_MC_FLOOD_DP_M	GENMASK(22, 16)
#define AR9344_SW_FLOOD_MASK_UC_FLOOD_DP_M	GENMASK(6, 0)

#define AR9331_SW_REG_GLOBAL_CTRL		0x30
#define AR9331_SW_GLOBAL_CTRL_MFS_M		GENMASK(13, 0)

#define AR9331_SW_REG_AT(_id)		(0x50 + (_id) * 4)
#define AR9331_SW_AT0_ADDR_BYTE_4	GENMASK(31, 24)
#define AR9331_SW_AT0_ADDR_BYTE_5	GENMASK(23, 16)
#define AR9331_SW_AT0_FULL		BIT(12)
#define AR9331_SW_AT0_BUSY		BIT(3)
#define AR9331_SW_AT0_FUNC		GENMASK(2, 0)

#define AR9331_SW_REG_AT_CTRL		0x5c
#define AR9331_SW_AT_CTRL_LEARN_CHANGE	BIT(18)
#define AR9331_SW_AT_CTRL_AGING		BIT(17)
#define AR9331_SW_AT_CTRL_AGE_TIME	GENMASK(15, 0)

#define AR9331_SW_AT1_ADDR_BYTE_0	GENMASK(31, 24)
#define AR9331_SW_AT1_ADDR_BYTE_1	GENMASK(23, 16)
#define AR9331_SW_AT1_ADDR_BYTE_2	GENMASK(15, 8)
#define AR9331_SW_AT1_ADDR_BYTE_3	GENMASK(7, 0)

#define AR9331_SW_AT2_AT_STATUS		GENMASK(19, 16)
#define AR9331_SW_AT2_REDIRECT_TO_CPU	BIT(25)
#define AR9331_SW_AT2_DES_PORT		GENMASK(6, 0)

#define AR9331_SW_AT3_AGE_TIME		GENMASK(15, 0)

#define AR9331_SW_REG_CPU_PORT			0x78
#define AR9344_SW_CPU_PORT_CPU_EN		BIT(8)

#define AR9331_SW_REG_MDIO_CTRL			0x98
#define AR9331_SW_MDIO_CTRL_BUSY		BIT(31)
#define AR9331_SW_MDIO_CTRL_MASTER_EN		BIT(30)
#define AR9331_SW_MDIO_CTRL_CMD_READ		BIT(27)
#define AR9331_SW_MDIO_CTRL_PHY_ADDR_M		GENMASK(25, 21)
#define AR9331_SW_MDIO_CTRL_REG_ADDR_M		GENMASK(20, 16)
#define AR9331_SW_MDIO_CTRL_DATA_M		GENMASK(16, 0)

#define AR9331_SW_REG_PORT_STATUS(_port)	(0x100 + (_port) * 0x100)

/* FLOW_LINK_EN - enable mac flow control config auto-neg with phy.
 * If not set, mac can be config by software.
 */
#define AR9331_SW_PORT_STATUS_FLOW_LINK_EN	BIT(12)

/* LINK_EN - If set, MAC is configured from PHY link status.
 * If not set, MAC should be configured by software.
 */
#define AR9331_SW_PORT_STATUS_LINK_EN		BIT(9)
#define AR9331_SW_PORT_STATUS_DUPLEX_MODE	BIT(6)
#define AR9331_SW_PORT_STATUS_RX_FLOW_EN	BIT(5)
#define AR9331_SW_PORT_STATUS_TX_FLOW_EN	BIT(4)
#define AR9331_SW_PORT_STATUS_RXMAC		BIT(3)
#define AR9331_SW_PORT_STATUS_TXMAC		BIT(2)
#define AR9331_SW_PORT_STATUS_SPEED_M		GENMASK(1, 0)
#define AR9331_SW_PORT_STATUS_SPEED_1000	2
#define AR9331_SW_PORT_STATUS_SPEED_100		1
#define AR9331_SW_PORT_STATUS_SPEED_10		0

#define AR9331_SW_PORT_STATUS_MAC_MASK \
	(AR9331_SW_PORT_STATUS_TXMAC | AR9331_SW_PORT_STATUS_RXMAC)

#define AR9331_SW_PORT_STATUS_LINK_MASK \
	(AR9331_SW_PORT_STATUS_DUPLEX_MODE | \
	 AR9331_SW_PORT_STATUS_RX_FLOW_EN | AR9331_SW_PORT_STATUS_TX_FLOW_EN | \
	 AR9331_SW_PORT_STATUS_SPEED_M)

#define AR9331_SW_REG_PORT_CTRL(_port)		(0x104 + (_port) * 0x100)
#define AR9344_SW_PORT_CTRL_QCA_HDR		BIT(11)

#define AR9344_SW_REG_PORT_VLAN1(_port)		(0x108 + (_port) * 0x100)
#define AR9344_SW_PORT_VLAN1_PORT_VLAN_EN	BIT(28)

#define AR9344_SW_REG_PORT_VLAN2(_port)		(0x10C + (_port) * 0x100)
#define AR9344_SW_PORT_VLAN2_PORT_VID_MEM_M	GENMASK(22, 16)
#define AR9344_SW_PORT_VLAN2_SET_PORTS(_ports) \
	(FIELD_PREP(AR9344_SW_PORT_VLAN2_PORT_VID_MEM_M, _ports))

/* Phy bypass mode
 * ------------------------------------------------------------------------
 * Bit:   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |
 *
 * real   | start |   OP  | PhyAddr           |  Reg Addr         |  TA   |
 * atheros| start |   OP  | 2'b00 |PhyAdd[2:0]|  Reg Addr[4:0]    |  TA   |
 *
 *
 * Bit:   |16 |17 |18 |19 |20 |21 |22 |23 |24 |25 |26 |27 |28 |29 |30 |31 |
 * real   |  Data                                                         |
 * atheros|  Data                                                         |
 *
 * ------------------------------------------------------------------------
 * Page address mode
 * ------------------------------------------------------------------------
 * Bit:   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |
 * real   | start |   OP  | PhyAddr           |  Reg Addr         |  TA   |
 * atheros| start |   OP  | 2'b11 |                          8'b0 |  TA   |
 *
 * Bit:   |16 |17 |18 |19 |20 |21 |22 |23 |24 |25 |26 |27 |28 |29 |30 |31 |
 * real   |  Data                                                         |
 * atheros|                       | Page [9:0]                            |
 */
/* In case of Page Address mode, Bit[18:9] of 32 bit register address should be
 * written to bits[9:0] of mdio data register.
 */
#define AR9331_SW_ADDR_PAGE			GENMASK(18, 9)

/* ------------------------------------------------------------------------
 * Normal register access mode
 * ------------------------------------------------------------------------
 * Bit:   | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |10 |11 |12 |13 |14 |15 |
 * real   | start |   OP  | PhyAddr           |  Reg Addr         |  TA   |
 * atheros| start |   OP  | 2'b10 |  low_addr[7:0]                |  TA   |
 *
 * Bit:   |16 |17 |18 |19 |20 |21 |22 |23 |24 |25 |26 |27 |28 |29 |30 |31 |
 * real   |  Data                                                         |
 * atheros|  Data                                                         |
 * ------------------------------------------------------------------------
 */
#define AR9331_SW_LOW_ADDR_PHY			GENMASK(8, 6)
#define AR9331_SW_LOW_ADDR_REG			GENMASK(5, 1)

#define AR9331_SW_MDIO_PHY_MODE_M		GENMASK(4, 3)
#define AR9331_SW_MDIO_PHY_MODE_PAGE		3
#define AR9331_SW_MDIO_PHY_MODE_REG		2
#define AR9331_SW_MDIO_PHY_MODE_BYPASS		0
#define AR9331_SW_MDIO_PHY_ADDR_M		GENMASK(2, 0)

/* Empirical determined values */
#define AR9331_SW_MDIO_POLL_SLEEP_US		10
#define AR9331_SW_MDIO_POLL_TIMEOUT_US		100

#define AR9331_SW_AT_POLL_SLEEP_US		1000
#define AR9331_SW_AT_POLL_TIMEOUT_US		5000


enum ar9331_fdb_cmd {
	AR9331_FDB_NOP = 0,
	AR9331_FDB_FLUSH = 1,
	AR9331_FDB_LOAD = 2,
	AR9331_FDB_PURGE = 3,
	AR9331_FDB_NEXT = 6,
	AR9331_FDB_SEARCH = 7,
};

struct ar9331_fdb {
	u16 vid;
	u8 port_mask;
	u8 aging;
	u8 mac[6];
};

struct ar9331_sw_priv {
	struct device *dev;
	struct dsa_switch ds;
	struct dsa_switch_ops ops;
	struct irq_domain *irqdomain;
	struct mii_bus *mbus; /* mdio master */
	struct mii_bus *sbus; /* mdio slave */
	struct regmap *regmap;
	struct reset_control *sw_reset;
	struct mutex reg_mutex;
};

/* Warning: switch reset will reset last AR9331_SW_MDIO_PHY_MODE_PAGE request
 * If some kind of optimization is used, the request should be repeated.
 */
static int ar9331_sw_reset(struct ar9331_sw_priv *priv)
{
	int ret;

	ret = reset_control_assert(priv->sw_reset);
	if (ret)
		goto error;

	/* AR9331 doc do not provide any information about proper reset
	 * sequence. The AR8136 (the closes switch to the AR9331) doc says:
	 * reset duration should be greater than 10ms. So, let's use this value
	 * for now.
	 */
	usleep_range(10000, 15000);
	ret = reset_control_deassert(priv->sw_reset);
	if (ret)
		goto error;
	/* There is no information on how long should we wait after reset.
	 * AR8136 has an EEPROM and there is an Interrupt for EEPROM load
	 * status. AR9331 has no EEPROM support.
	 * For now, do not wait. In case AR8136 will be needed, the after
	 * reset delay can be added as well.
	 */

	return 0;
error:
	dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
	return ret;
}

static int ar9331_sw_mbus_write(struct mii_bus *mbus, int port, int regnum,
				u16 data)
{
	struct ar9331_sw_priv *priv = mbus->priv;
	struct regmap *regmap = priv->regmap;
	u32 val;
	int ret;

	ret = regmap_write(regmap, AR9331_SW_REG_MDIO_CTRL,
			   AR9331_SW_MDIO_CTRL_BUSY |
			   AR9331_SW_MDIO_CTRL_MASTER_EN |
			   FIELD_PREP(AR9331_SW_MDIO_CTRL_PHY_ADDR_M, port) |
			   FIELD_PREP(AR9331_SW_MDIO_CTRL_REG_ADDR_M, regnum) |
			   FIELD_PREP(AR9331_SW_MDIO_CTRL_DATA_M, data));
	if (ret)
		goto error;

	ret = regmap_read_poll_timeout(regmap, AR9331_SW_REG_MDIO_CTRL, val,
				       !(val & AR9331_SW_MDIO_CTRL_BUSY),
				       AR9331_SW_MDIO_POLL_SLEEP_US,
				       AR9331_SW_MDIO_POLL_TIMEOUT_US);
	if (ret)
		goto error;

	return 0;
error:
	dev_err_ratelimited(priv->dev, "PHY write error: %i\n", ret);
	return ret;
}

static int ar9331_sw_mbus_read(struct mii_bus *mbus, int port, int regnum)
{
	struct ar9331_sw_priv *priv = mbus->priv;
	struct regmap *regmap = priv->regmap;
	u32 val;
	int ret;

	ret = regmap_write(regmap, AR9331_SW_REG_MDIO_CTRL,
			   AR9331_SW_MDIO_CTRL_BUSY |
			   AR9331_SW_MDIO_CTRL_MASTER_EN |
			   AR9331_SW_MDIO_CTRL_CMD_READ |
			   FIELD_PREP(AR9331_SW_MDIO_CTRL_PHY_ADDR_M, port) |
			   FIELD_PREP(AR9331_SW_MDIO_CTRL_REG_ADDR_M, regnum));
	if (ret)
		goto error;

	ret = regmap_read_poll_timeout(regmap, AR9331_SW_REG_MDIO_CTRL, val,
				       !(val & AR9331_SW_MDIO_CTRL_BUSY),
				       AR9331_SW_MDIO_POLL_SLEEP_US,
				       AR9331_SW_MDIO_POLL_TIMEOUT_US);
	if (ret)
		goto error;

	ret = regmap_read(regmap, AR9331_SW_REG_MDIO_CTRL, &val);
	if (ret)
		goto error;

	return FIELD_GET(AR9331_SW_MDIO_CTRL_DATA_M, val);

error:
	dev_err_ratelimited(priv->dev, "PHY read error: %i\n", ret);
	return ret;
}

static int ar9331_sw_mbus_init(struct ar9331_sw_priv *priv)
{
	struct device *dev = priv->dev;
	struct mii_bus *mbus;
	struct device_node *np, *mnp;
	int ret;

	np = dev->of_node;

	mbus = devm_mdiobus_alloc(dev);
	if (!mbus)
		return -ENOMEM;

	mbus->name = np->full_name;
	snprintf(mbus->id, MII_BUS_ID_SIZE, "%pOF", np);

	mbus->read = ar9331_sw_mbus_read;
	mbus->write = ar9331_sw_mbus_write;
	mbus->priv = priv;
	mbus->parent = dev;

	mnp = of_get_child_by_name(np, "mdio");
	if (!mnp)
		return -ENODEV;

	ret = of_mdiobus_register(mbus, mnp);
	of_node_put(mnp);
	if (ret)
		return ret;

	priv->mbus = mbus;

	return 0;
}

static int ar9331_sw_setup(struct dsa_switch *ds)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = ar9331_sw_reset(priv);
	if (ret)
		return ret;

	/* Reset will set proper defaults. CPU - Port0 will be enabled and
	 * configured. All other ports (ports 1 - 5) are disabled
	 */
	ret = ar9331_sw_mbus_init(priv);
	if (ret)
		return ret;

	/* Do not drop broadcast frames */
	ret = regmap_write_bits(regmap, AR9331_SW_REG_FLOOD_MASK,
				AR9331_SW_FLOOD_MASK_BROAD_TO_CPU,
				AR9331_SW_FLOOD_MASK_BROAD_TO_CPU);
	if (ret)
		goto error;

	/* Set max frame size to the maximum supported value */
	ret = regmap_write_bits(regmap, AR9331_SW_REG_GLOBAL_CTRL,
				AR9331_SW_GLOBAL_CTRL_MFS_M,
				AR9331_SW_GLOBAL_CTRL_MFS_M);
	if (ret)
		goto error;

	return 0;
error:
	dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
	return ret;
}

static int ar9344_sw_setup(struct dsa_switch *ds)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;
	int i;

	/* CPU port has to be Port 0 */
	if (!dsa_is_cpu_port(ds, AR9331_CPU_PORT)) {
		pr_err("port 0 is not the CPU port\n");
		return -EINVAL;
	}

	ret = ar9331_sw_reset(priv);
	if (ret)
		return ret;

	ret = ar9331_sw_mbus_init(priv);
	if (ret)
		return ret;

	/* Enable CPU port */
	ret = regmap_update_bits(regmap, AR9331_SW_REG_CPU_PORT,
				 AR9344_SW_CPU_PORT_CPU_EN,
				 AR9344_SW_CPU_PORT_CPU_EN);
	if (ret)
		goto error;

	/* Enable QCA header on CPU port */
	ret = regmap_update_bits(regmap, AR9331_SW_REG_PORT_CTRL(0),
				 AR9344_SW_PORT_CTRL_QCA_HDR,
				 AR9344_SW_PORT_CTRL_QCA_HDR);
	if (ret)
		goto error;

	/* Set max frame size to the maximum supported value */
	ret = regmap_update_bits(regmap, AR9331_SW_REG_GLOBAL_CTRL,
				 AR9331_SW_GLOBAL_CTRL_MFS_M,
				 AR9331_SW_GLOBAL_CTRL_MFS_M);
	if (ret)
		goto error;

	/* Forward all unknown frames to CPU port for Linux processing */
	ret = regmap_update_bits(regmap, AR9331_SW_REG_FLOOD_MASK,
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_BC_DP_M, BIT(AR9331_CPU_PORT)) |
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_MC_FLOOD_DP_M, BIT(AR9331_CPU_PORT)) |
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_UC_FLOOD_DP_M, BIT(AR9331_CPU_PORT)),
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_BC_DP_M, BIT(AR9331_CPU_PORT)) |
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_MC_FLOOD_DP_M, BIT(AR9331_CPU_PORT)) |
				 FIELD_PREP(AR9344_SW_FLOOD_MASK_UC_FLOOD_DP_M, BIT(AR9331_CPU_PORT)));
	if (ret)
		goto error;

	ret = regmap_write(regmap, AR9331_SW_REG_AT_CTRL,
			  FIELD_PREP(AR9331_SW_AT_CTRL_AGE_TIME, 0x2b) | /* 5 min age time */
			  AR9331_SW_AT_CTRL_LEARN_CHANGE |
			  AR9331_SW_AT_CTRL_AGING);
	if (ret)
		goto error;

	/* Setup connection between CPU port & user ports */
	for (i = 0; i < AR9331_SW_PORTS; i++) {
		/* Enable Port-based VLAN */
		ret = regmap_update_bits(regmap, AR9344_SW_REG_PORT_VLAN1(i),
					 AR9344_SW_PORT_VLAN1_PORT_VLAN_EN,
					 AR9344_SW_PORT_VLAN1_PORT_VLAN_EN);
		if (ret)
			goto error;

		/* CPU port is already connected to all ports */
		if (dsa_is_cpu_port(ds, i))
			continue;

		/* Individual user ports get connected to CPU port only */
		if (dsa_is_user_port(ds, i)) {
			ret = regmap_update_bits(regmap, AR9344_SW_REG_PORT_VLAN2(i),
						 AR9344_SW_PORT_VLAN2_PORT_VID_MEM_M,
						 AR9344_SW_PORT_VLAN2_SET_PORTS(BIT(AR9331_CPU_PORT)));
			if (ret)
				goto error;
		}
	}

	return 0;
error:
	dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
	return ret;
}

static void ar9331_sw_port_disable(struct dsa_switch *ds, int port)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = regmap_write(regmap, AR9331_SW_REG_PORT_STATUS(port), 0);
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static enum dsa_tag_protocol ar9331_sw_get_tag_protocol(struct dsa_switch *ds,
							int port,
							enum dsa_tag_protocol m)
{
	return DSA_TAG_PROTO_AR9331;
}

static enum dsa_tag_protocol ar9344_sw_get_tag_protocol(struct dsa_switch *ds,
							int port,
							enum dsa_tag_protocol m)
{
	return DSA_TAG_PROTO_AR9344;
}

static int wait_atu_ready(struct ar9331_sw_priv *priv)
{
	struct regmap *regmap = priv->regmap;
	u32 reg;

	return regmap_read_poll_timeout(regmap, AR9331_SW_REG_AT(0), reg,
			!(reg & AR9331_SW_AT0_BUSY),
			AR9331_SW_AT_POLL_SLEEP_US,
			AR9331_SW_AT_POLL_TIMEOUT_US);	
}

static int ar9331_fdb_next(struct ar9331_sw_priv *priv, struct ar9331_fdb *fdb)
{
	struct regmap *regmap = priv->regmap;
	u32 reg[3] = { 0 };
	int ret;
	int i;

	ret = wait_atu_ready(priv);
	if (ret)
		goto error;

	if (!fdb->aging) {
		/* Clear only for the first entry */
		reg[0] = FIELD_PREP(AR9331_SW_AT0_FUNC, AR9331_FDB_NEXT);
		regmap_write(regmap, AR9331_SW_REG_AT(0), reg[0]);
		regmap_write(regmap, AR9331_SW_REG_AT(2), reg[2]);
		regmap_write(regmap, AR9331_SW_REG_AT(1), reg[1]);
	}

	regmap_write_bits(regmap, AR9331_SW_REG_AT(0),
			  AR9331_SW_AT0_BUSY,
			  AR9331_SW_AT0_BUSY);


	usleep_range(10000, 20000);
	ret = wait_atu_ready(priv);
	if (ret)
		goto error;

	for (i = 0; i < 3; i++) {
		ret = regmap_read(regmap, AR9331_SW_REG_AT(i), &reg[i]);
		if (ret)
			goto error;
	}

	fdb->vid = 0;
	/* aging - 67:64 */
	fdb->aging = FIELD_GET(AR9331_SW_AT2_AT_STATUS, reg[2]);
	/* portmask - 54:48 */
	fdb->port_mask = FIELD_GET(AR9331_SW_AT2_DES_PORT, reg[2]);
	/* mac - 47:0 */
	fdb->mac[0] = FIELD_GET(AR9331_SW_AT1_ADDR_BYTE_0, reg[1]);
	fdb->mac[1] = FIELD_GET(AR9331_SW_AT1_ADDR_BYTE_1, reg[1]);
	fdb->mac[2] = FIELD_GET(AR9331_SW_AT1_ADDR_BYTE_2, reg[1]);
	fdb->mac[3] = FIELD_GET(AR9331_SW_AT1_ADDR_BYTE_3, reg[1]);
	fdb->mac[4] = FIELD_GET(AR9331_SW_AT0_ADDR_BYTE_4, reg[0]);
	fdb->mac[5] = FIELD_GET(AR9331_SW_AT0_ADDR_BYTE_5, reg[0]);

	pr_warn("R0 %08x R1 %08x R2 %08x", reg[0], reg[1], reg[2]);

error:
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);

	return ret;
}

static int ar9331_port_fdb_dump(struct dsa_switch *ds, int port,
				dsa_fdb_dump_cb_t *cb, void *data)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	int cnt = 16;
	struct ar9331_fdb _fdb = { 0 };
	bool is_static;
	int ret = 0;

	mutex_lock(&priv->reg_mutex);

	while (cnt > 0 && !ar9331_fdb_next(priv, &_fdb)) {
		cnt--;
		if (!_fdb.aging)
			break;
		if (!(_fdb.port_mask & BIT(port)))
			continue;
		is_static = (_fdb.aging == 0xf);
		ret = cb(_fdb.mac, _fdb.vid, is_static, data);
		if (ret)
			goto error;
	}

	mutex_unlock(&priv->reg_mutex);

	pr_warn("Count %d entries in the ARL table", cnt);

error:
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
	return ret;
}

static int ar9344_port_bridge_join(struct dsa_switch *ds, int port,
				   struct net_device *br)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int port_mask = BIT(AR9331_CPU_PORT);
	int ret;
	int i;

	for (i = 1; i < AR9331_SW_PORTS; i++) {
		if (dsa_to_port(ds, i)->bridge_dev != br)
			continue;
		/* Add this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		ret = regmap_update_bits(regmap,
					 AR9344_SW_REG_PORT_VLAN2(i),
					 AR9344_SW_PORT_VLAN2_SET_PORTS(BIT(port)),
					 AR9344_SW_PORT_VLAN2_SET_PORTS(BIT(port)));
		if (ret)
			goto error;

		if (i != port)
			port_mask |= BIT(i);
	}
	/* Add all other ports to this ports portvlan mask */
	ret = regmap_update_bits(regmap, AR9344_SW_REG_PORT_VLAN2(port),
				 AR9344_SW_PORT_VLAN2_PORT_VID_MEM_M, 
				 AR9344_SW_PORT_VLAN2_SET_PORTS(port_mask));
	if (ret)
		goto error;

	return 0;
error:
	dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
	return ret;
}

static void ar9344_port_bridge_leave(struct dsa_switch *ds, int port,
				    struct net_device *br)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;
	int i;

	for (i = 1; i < AR9331_SW_PORTS; i++) {
		if (dsa_to_port(ds, i)->bridge_dev != br)
			continue;
		/* Remove this port to the portvlan mask of the other ports
		 * in the bridge
		 */
		ret = regmap_update_bits(regmap,
					 AR9344_SW_REG_PORT_VLAN2(i),
					 AR9344_SW_PORT_VLAN2_SET_PORTS(BIT(port)),
					 0);
		if (ret)
			goto error;
	}

	/* Set the cpu port to be the only one in the portvlan mask of
	 * this port
	 */
	ret = regmap_update_bits(regmap, AR9344_SW_REG_PORT_VLAN2(port),
				 AR9344_SW_PORT_VLAN2_PORT_VID_MEM_M,
				 AR9344_SW_PORT_VLAN2_SET_PORTS(BIT(AR9331_CPU_PORT)));
error:
	dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static void ar9331_sw_phylink_validate(struct dsa_switch *ds, int port,
				       unsigned long *supported,
				       struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	switch (port) {
	case 0:
		if (state->interface != PHY_INTERFACE_MODE_GMII)
			goto unsupported;

		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseT_Half);
		break;
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
		if (state->interface != PHY_INTERFACE_MODE_INTERNAL)
			goto unsupported;
		break;
	default:
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev, "Unsupported port: %i\n", port);
		return;
	}

	phylink_set_port_modes(mask);
	phylink_set(mask, Pause);
	phylink_set(mask, Asym_Pause);

	phylink_set(mask, 10baseT_Half);
	phylink_set(mask, 10baseT_Full);
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);

	return;

unsupported:
	bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
	dev_err(ds->dev, "Unsupported interface: %d, port: %d\n",
		state->interface, port);
}

static void ar9331_sw_phylink_mac_config(struct dsa_switch *ds, int port,
					 unsigned int mode,
					 const struct phylink_link_state *state)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = regmap_update_bits(regmap, AR9331_SW_REG_PORT_STATUS(port),
				 AR9331_SW_PORT_STATUS_LINK_EN |
				 AR9331_SW_PORT_STATUS_FLOW_LINK_EN, 0);
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static void ar9331_sw_phylink_mac_link_down(struct dsa_switch *ds, int port,
					    unsigned int mode,
					    phy_interface_t interface)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = regmap_update_bits(regmap, AR9331_SW_REG_PORT_STATUS(port),
				 AR9331_SW_PORT_STATUS_MAC_MASK, 0);
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static void ar9331_sw_phylink_mac_link_up(struct dsa_switch *ds, int port,
					  unsigned int mode,
					  phy_interface_t interface,
					  struct phy_device *phydev,
					  int speed, int duplex,
					  bool tx_pause, bool rx_pause)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ds->priv;
	struct regmap *regmap = priv->regmap;
	u32 val;
	int ret;

	val = AR9331_SW_PORT_STATUS_MAC_MASK;
	switch (speed) {
	case SPEED_1000:
		val |= AR9331_SW_PORT_STATUS_SPEED_1000;
		break;
	case SPEED_100:
		val |= AR9331_SW_PORT_STATUS_SPEED_100;
		break;
	case SPEED_10:
		val |= AR9331_SW_PORT_STATUS_SPEED_10;
		break;
	default:
		return;
	}

	if (duplex)
		val |= AR9331_SW_PORT_STATUS_DUPLEX_MODE;

	if (tx_pause)
		val |= AR9331_SW_PORT_STATUS_TX_FLOW_EN;

	if (rx_pause)
		val |= AR9331_SW_PORT_STATUS_RX_FLOW_EN;

	ret = regmap_update_bits(regmap, AR9331_SW_REG_PORT_STATUS(port),
				 AR9331_SW_PORT_STATUS_MAC_MASK |
				 AR9331_SW_PORT_STATUS_LINK_MASK,
				 val);
	if (ret)
		dev_err_ratelimited(priv->dev, "%s: %i\n", __func__, ret);
}

static const struct dsa_switch_ops ar9331_sw_ops = {
	.get_tag_protocol	= ar9331_sw_get_tag_protocol,
	.setup			= ar9331_sw_setup,
	.port_disable		= ar9331_sw_port_disable,
	.phylink_validate	= ar9331_sw_phylink_validate,
	.phylink_mac_config	= ar9331_sw_phylink_mac_config,
	.phylink_mac_link_down	= ar9331_sw_phylink_mac_link_down,
	.phylink_mac_link_up	= ar9331_sw_phylink_mac_link_up,
};

static const struct dsa_switch_ops ar9344_sw_ops = {
	.get_tag_protocol	= ar9344_sw_get_tag_protocol,
	.setup			= ar9344_sw_setup,
	.port_disable		= ar9331_sw_port_disable,
	.port_bridge_join	= ar9344_port_bridge_join,
	.port_bridge_leave	= ar9344_port_bridge_leave,
	.port_fdb_dump		= ar9331_port_fdb_dump,
	.phylink_validate	= ar9331_sw_phylink_validate,
	.phylink_mac_config	= ar9331_sw_phylink_mac_config,
	.phylink_mac_link_down	= ar9331_sw_phylink_mac_link_down,
	.phylink_mac_link_up	= ar9331_sw_phylink_mac_link_up,
};

static irqreturn_t ar9331_sw_irq(int irq, void *data)
{
	struct ar9331_sw_priv *priv = data;
	struct regmap *regmap = priv->regmap;
	u32 stat;
	int ret;

	ret = regmap_read(regmap, AR9331_SW_REG_GINT, &stat);
	if (ret) {
		dev_err(priv->dev, "can't read interrupt status\n");
		return IRQ_NONE;
	}

	if (!stat)
		return IRQ_NONE;

	if (stat & AR9331_SW_GINT_PHY_INT) {
		int child_irq;

		child_irq = irq_find_mapping(priv->irqdomain, 0);
		handle_nested_irq(child_irq);
	}

	ret = regmap_write(regmap, AR9331_SW_REG_GINT, stat);
	if (ret) {
		dev_err(priv->dev, "can't write interrupt status\n");
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static void ar9331_sw_mask_irq(struct irq_data *d)
{
	struct ar9331_sw_priv *priv = irq_data_get_irq_chip_data(d);
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = regmap_update_bits(regmap, AR9331_SW_REG_GINT_MASK,
				 AR9331_SW_GINT_PHY_INT, 0);
	if (ret)
		dev_err(priv->dev, "could not mask IRQ\n");
}

static void ar9331_sw_unmask_irq(struct irq_data *d)
{
	struct ar9331_sw_priv *priv = irq_data_get_irq_chip_data(d);
	struct regmap *regmap = priv->regmap;
	int ret;

	ret = regmap_update_bits(regmap, AR9331_SW_REG_GINT_MASK,
				 AR9331_SW_GINT_PHY_INT,
				 AR9331_SW_GINT_PHY_INT);
	if (ret)
		dev_err(priv->dev, "could not unmask IRQ\n");
}

static struct irq_chip ar9331_sw_irq_chip = {
	.name = AR9331_SW_NAME,
	.irq_mask = ar9331_sw_mask_irq,
	.irq_unmask = ar9331_sw_unmask_irq,
};

static int ar9331_sw_irq_map(struct irq_domain *domain, unsigned int irq,
			     irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, domain->host_data);
	irq_set_chip_and_handler(irq, &ar9331_sw_irq_chip, handle_simple_irq);
	irq_set_nested_thread(irq, 1);
	irq_set_noprobe(irq);

	return 0;
}

static void ar9331_sw_irq_unmap(struct irq_domain *d, unsigned int irq)
{
	irq_set_nested_thread(irq, 0);
	irq_set_chip_and_handler(irq, NULL, NULL);
	irq_set_chip_data(irq, NULL);
}

static const struct irq_domain_ops ar9331_sw_irqdomain_ops = {
	.map = ar9331_sw_irq_map,
	.unmap = ar9331_sw_irq_unmap,
	.xlate = irq_domain_xlate_onecell,
};

static int ar9331_sw_irq_init(struct ar9331_sw_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	struct device *dev = priv->dev;
	int ret, irq;

	irq = of_irq_get(np, 0);
	if (irq <= 0) {
		dev_err(dev, "failed to get parent IRQ\n");
		return irq ? irq : -EINVAL;
	}

	ret = devm_request_threaded_irq(dev, irq, NULL, ar9331_sw_irq,
					IRQF_ONESHOT, AR9331_SW_NAME, priv);
	if (ret) {
		dev_err(dev, "unable to request irq: %d\n", ret);
		return ret;
	}

	priv->irqdomain = irq_domain_add_linear(np, 1, &ar9331_sw_irqdomain_ops,
						priv);
	if (!priv->irqdomain) {
		dev_err(dev, "failed to create IRQ domain\n");
		return -EINVAL;
	}

	irq_set_parent(irq_create_mapping(priv->irqdomain, 0), irq);

	return 0;
}

static int __ar9331_mdio_write(struct mii_bus *sbus, u8 mode, u16 reg, u16 val)
{
	u8 r, p;

	p = FIELD_PREP(AR9331_SW_MDIO_PHY_MODE_M, mode) |
		FIELD_GET(AR9331_SW_LOW_ADDR_PHY, reg);
	r = FIELD_GET(AR9331_SW_LOW_ADDR_REG, reg);

	return mdiobus_write(sbus, p, r, val);
}

static int __ar9331_mdio_read(struct mii_bus *sbus, u16 reg)
{
	u8 r, p;

	p = FIELD_PREP(AR9331_SW_MDIO_PHY_MODE_M, AR9331_SW_MDIO_PHY_MODE_REG) |
		FIELD_GET(AR9331_SW_LOW_ADDR_PHY, reg);
	r = FIELD_GET(AR9331_SW_LOW_ADDR_REG, reg);

	return mdiobus_read(sbus, p, r);
}

static int ar9331_mdio_read(void *ctx, const void *reg_buf, size_t reg_len,
			    void *val_buf, size_t val_len)
{
	struct ar9331_sw_priv *priv = ctx;
	struct mii_bus *sbus = priv->sbus;
	u32 reg = *(u32 *)reg_buf;
	int ret;

	if (reg == AR9331_SW_REG_PAGE) {
		/* We cannot read the page selector register from hardware and
		 * we cache its value in regmap. Return all bits set here,
		 * that regmap will always write the page on first use.
		 */
		*(u32 *)val_buf = GENMASK(9, 0);
		return 0;
	}

	ret = __ar9331_mdio_read(sbus, reg);
	if (ret < 0)
		goto error;

	*(u32 *)val_buf = ret;
	ret = __ar9331_mdio_read(sbus, reg + 2);
	if (ret < 0)
		goto error;

	*(u32 *)val_buf |= ret << 16;

	return 0;
error:
	dev_err_ratelimited(&sbus->dev, "Bus error. Failed to read register.\n");
	return ret;
}

static int ar9331_mdio_write(void *ctx, u32 reg, u32 val)
{
	struct ar9331_sw_priv *priv = (struct ar9331_sw_priv *)ctx;
	struct mii_bus *sbus = priv->sbus;
	int ret;

	if (reg == AR9331_SW_REG_PAGE) {
		ret = __ar9331_mdio_write(sbus, AR9331_SW_MDIO_PHY_MODE_PAGE,
					  0, val);
		if (ret < 0)
			goto error;

		return 0;
	}

	ret = __ar9331_mdio_write(sbus, AR9331_SW_MDIO_PHY_MODE_REG, reg, val);
	if (ret < 0)
		goto error;

	ret = __ar9331_mdio_write(sbus, AR9331_SW_MDIO_PHY_MODE_REG, reg + 2,
				  val >> 16);
	if (ret < 0)
		goto error;

	return 0;
error:
	dev_err_ratelimited(&sbus->dev, "Bus error. Failed to write register.\n");
	return ret;
}

static int ar9331_sw_bus_write(void *context, const void *data, size_t count)
{
	u32 reg = *(u32 *)data;
	u32 val = *((u32 *)data + 1);

	return ar9331_mdio_write(context, reg, val);
}

static const struct regmap_range ar9331_valid_regs[] = {
	regmap_reg_range(0x0, 0x0),
	regmap_reg_range(0x10, 0x14),
	regmap_reg_range(0x20, 0x24),
	regmap_reg_range(0x2c, 0x30),
	regmap_reg_range(0x40, 0x44),
	regmap_reg_range(0x50, 0x78),
	regmap_reg_range(0x80, 0x98),

	regmap_reg_range(0x100, 0x120),
	regmap_reg_range(0x200, 0x220),
	regmap_reg_range(0x300, 0x320),
	regmap_reg_range(0x400, 0x420),
	regmap_reg_range(0x500, 0x520),
	regmap_reg_range(0x600, 0x620),

	regmap_reg_range(0x20000, 0x200a4),
	regmap_reg_range(0x20100, 0x201a4),
	regmap_reg_range(0x20200, 0x202a4),
	regmap_reg_range(0x20300, 0x203a4),
	regmap_reg_range(0x20400, 0x204a4),
	regmap_reg_range(0x20500, 0x205a4),

	/* dummy page selector reg */
	regmap_reg_range(AR9331_SW_REG_PAGE, AR9331_SW_REG_PAGE),
};

static const struct regmap_range ar9331_nonvolatile_regs[] = {
	regmap_reg_range(AR9331_SW_REG_PAGE, AR9331_SW_REG_PAGE),
};

static const struct regmap_range_cfg ar9331_regmap_range[] = {
	{
		.selector_reg = AR9331_SW_REG_PAGE,
		.selector_mask = GENMASK(9, 0),
		.selector_shift = 0,

		.window_start = 0,
		.window_len = 512,

		.range_min = 0,
		.range_max = AR9331_SW_REG_PAGE - 4,
	},
};

static const struct regmap_access_table ar9331_register_set = {
	.yes_ranges = ar9331_valid_regs,
	.n_yes_ranges = ARRAY_SIZE(ar9331_valid_regs),
};

static const struct regmap_access_table ar9331_volatile_set = {
	.no_ranges = ar9331_nonvolatile_regs,
	.n_no_ranges = ARRAY_SIZE(ar9331_nonvolatile_regs),
};

static const struct regmap_config ar9331_mdio_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = AR9331_SW_REG_PAGE,

	.ranges = ar9331_regmap_range,
	.num_ranges = ARRAY_SIZE(ar9331_regmap_range),

	.volatile_table = &ar9331_volatile_set,
	.wr_table = &ar9331_register_set,
	.rd_table = &ar9331_register_set,

	.cache_type = REGCACHE_RBTREE,
};

static struct regmap_bus ar9331_sw_bus = {
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.read = ar9331_mdio_read,
	.write = ar9331_sw_bus_write,
	.max_raw_read = 4,
	.max_raw_write = 4,
};

static int ar9331_sw_probe(struct mdio_device *mdiodev)
{
	struct ar9331_sw_priv *priv;
	struct dsa_switch *ds;
	int ret;

	priv = devm_kzalloc(&mdiodev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->reg_mutex);

	priv->regmap = devm_regmap_init(&mdiodev->dev, &ar9331_sw_bus, priv,
					&ar9331_mdio_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&mdiodev->dev, "regmap init failed: %d\n", ret);
		return ret;
	}

	priv->sw_reset = devm_reset_control_get(&mdiodev->dev, "switch");
	if (IS_ERR(priv->sw_reset)) {
		dev_err(&mdiodev->dev, "missing switch reset\n");
		return PTR_ERR(priv->sw_reset);
	}

	priv->sbus = mdiodev->bus;
	priv->dev = &mdiodev->dev;

	ret = ar9331_sw_irq_init(priv);
	if (ret)
		return ret;

	ds = &priv->ds;
	ds->dev = &mdiodev->dev;
	ds->num_ports = AR9331_SW_PORTS;
	ds->priv = priv;

	if (of_device_is_compatible(ds->dev->of_node, "qca,ar9344-switch"))
		priv->ops = ar9344_sw_ops;
	else
		priv->ops = ar9331_sw_ops;
	ds->ops = &priv->ops;
	dev_set_drvdata(&mdiodev->dev, priv);

	ret = dsa_register_switch(ds);
	if (ret)
		goto err_remove_irq;

	return 0;

err_remove_irq:
	irq_domain_remove(priv->irqdomain);

	return ret;
}

static void ar9331_sw_remove(struct mdio_device *mdiodev)
{
	struct ar9331_sw_priv *priv = dev_get_drvdata(&mdiodev->dev);

	irq_domain_remove(priv->irqdomain);
	mdiobus_unregister(priv->mbus);
	dsa_unregister_switch(&priv->ds);

	reset_control_assert(priv->sw_reset);
}

static const struct of_device_id ar9331_sw_of_match[] = {
	{ .compatible = "qca,ar9331-switch" },
	{ .compatible = "qca,ar9344-switch" },
	{ },
};

static struct mdio_driver ar9331_sw_mdio_driver = {
	.probe = ar9331_sw_probe,
	.remove = ar9331_sw_remove,
	.mdiodrv.driver = {
		.name = AR9331_SW_NAME,
		.of_match_table = ar9331_sw_of_match,
	},
};

mdio_module_driver(ar9331_sw_mdio_driver);

MODULE_AUTHOR("Oleksij Rempel <kernel@pengutronix.de>");
MODULE_DESCRIPTION("Driver for Atheros AR9331 switch");
MODULE_LICENSE("GPL v2");
