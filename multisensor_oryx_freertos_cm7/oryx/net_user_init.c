/**
 * @file net_user_init.h
 * @brief Network initialization
 * @author Oryx Embedded SARL (www.oryx-embedded.com)
 * @version 2.3.2
 **/

//Dependencies
#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_iomuxc.h"
#include "core/net.h"
#include "drivers/mac/mimxrt1170_eth1_driver.h"
#include "drivers/mac/mimxrt1170_eth2_driver.h"
#include "drivers/switch/ksz9563_driver.h"
#include "rstp/rstp.h"
#include "dhcp/dhcp_client.h"
#include "ipv6/slaac.h"
#include "http/http_server.h"
#include "snmp/snmp_agent.h"
#include "mibs/snmp_mib_module.h"
#include "mibs/snmp_mib_impl.h"
#include "mibs/bridge_mib_module.h"
#include "mibs/bridge_mib_impl.h"
#include "mibs/rstp_mib_module.h"
#include "mibs/rstp_mib_impl.h"
#include "hardware/mimxrt1170/mimxrt1170_crypto.h"
#include "rng/trng.h"
#include "rng/yarrow.h"
#include "path.h"
#include "date_time.h"
#include "resource_manager.h"
#include "http_server_callbacks.h"
#include "tls_server_demo.h"
#include "spi_driver.h"
#include "debug.h"

//Ethernet interface configuration
#define APP_NAME "eth0"
#define APP_HOST_NAME "https-server-demo"
#define APP_MAC_ADDR "00-AB-CD-11-76-00"

#define APP_USE_DHCP_CLIENT DISABLED
#define APP_IPV4_HOST_ADDR "192.168.0.21"
#define APP_IPV4_SUBNET_MASK "255.255.255.0"
#define APP_IPV4_DEFAULT_GATEWAY "192.168.0.254"
#define APP_IPV4_PRIMARY_DNS "8.8.8.8"
#define APP_IPV4_SECONDARY_DNS "8.8.4.4"

#define APP_USE_SLAAC ENABLED
#define APP_IPV6_LINK_LOCAL_ADDR "fe80::1176:1"

//RSTP configuration
#define RSTP_BRIDGE_NUM_PORTS 2

//HTTP server configuration
#define APP_HTTP_MAX_CONNECTIONS 8

//Global variables
RstpBridgeSettings rstpBridgeSettings;
RstpBridgeContext rstpBridgeContext;
RstpBridgePort rstpBridgePorts[RSTP_BRIDGE_NUM_PORTS];
DhcpClientSettings dhcpClientSettings;
DhcpClientContext dhcpClientContext;
SlaacSettings slaacSettings;
SlaacContext slaacContext;
HttpServerSettings httpServerSettings;
HttpServerContext httpServerContext;
HttpConnection httpConnections[APP_HTTP_MAX_CONNECTIONS];
SnmpAgentSettings snmpAgentSettings;
SnmpAgentContext snmpAgentContext;
YarrowContext yarrowContext;
TlsCache *tlsCache;
uint8_t seed[32];

NetInterface *interface;


/**
 * @brief MII/RMII/RGMII GPIO configuration
 * @param[in] interface Underlying network interface
 **/

void mimxrt1170Eth2InitGpio(NetInterface *interface)
{
   clock_sys_pll1_config_t sysPll1Config = {0};
   clock_root_config_t rootConfig = {0};

   //Initialize system PLL1
   sysPll1Config.pllDiv2En = true;
   CLOCK_InitSysPll1(&sysPll1Config);

   //Generate 125MHz root clock
   rootConfig.clockOff = false;
   rootConfig.mux = kCLOCK_ENET2_ClockRoot_MuxSysPll1Div2;
   rootConfig.div = 4;
   CLOCK_SetRootClock(kCLOCK_Root_Enet2, &rootConfig);

   //Initialize PLL PFD3 (528*18/24 = 396MHz)
   CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd3, 24);

   //Generate 198MHz bus clock
   rootConfig.clockOff = false;
   rootConfig.mux = kCLOCK_BUS_ClockRoot_MuxSysPll2Pfd3;
   rootConfig.div = 2;
   CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootConfig);

   //ENET_1G_TX_CLK is driven by ENET2_CLK_ROOT
   IOMUXC_GPR->GPR5 &= ~IOMUXC_GPR_GPR5_ENET1G_TX_CLK_SEL_MASK;
   //Enable ENET_1G_TX_CLK output
   IOMUXC_GPR->GPR5 |= IOMUXC_GPR_GPR5_ENET1G_RGMII_EN_MASK;

   //Enable IOMUXC clock
   CLOCK_EnableClock(kCLOCK_Iomuxc);

   //Configure GPIO_DISP_B1_00 pin as ENET_1G_RX_EN
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_00_ENET_1G_RX_EN, 0);

   //Set GPIO_DISP_B1_00 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_00_ENET_1G_RX_EN,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_01 pin as ENET_1G_RX_CLK
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_01_ENET_1G_RX_CLK, 0);

   //Set GPIO_DISP_B1_01 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_01_ENET_1G_RX_CLK,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_02 pin as ENET_1G_RX_DATA00
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_02_ENET_1G_RX_DATA00, 0);

   //Set GPIO_DISP_B1_02 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_02_ENET_1G_RX_DATA00,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_03 pin as ENET_1G_RX_DATA01
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_03_ENET_1G_RX_DATA01, 0);

   //Set GPIO_DISP_B1_03 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_03_ENET_1G_RX_DATA01,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_04 pin as ENET_1G_RX_DATA02
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_04_ENET_1G_RX_DATA02, 0);

   //Set GPIO_DISP_B1_04 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_04_ENET_1G_RX_DATA02,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_05 pin as ENET_1G_RX_DATA03
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_05_ENET_1G_RX_DATA03, 0);

   //Set GPIO_DISP_B1_05 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_05_ENET_1G_RX_DATA03,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(2) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_06 pin as ENET_1G_TX_DATA03
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_06_ENET_1G_TX_DATA03, 0);

   //Set GPIO_DISP_B1_06 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_06_ENET_1G_TX_DATA03,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_07 pin as ENET_1G_TX_DATA02
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_07_ENET_1G_TX_DATA02, 0);

   //Set GPIO_DISP_B1_07 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_07_ENET_1G_TX_DATA02,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_08 pin as ENET_1G_TX_DATA01
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_08_ENET_1G_TX_DATA01, 0);

   //Set GPIO_DISP_B1_08 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_08_ENET_1G_TX_DATA01,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_09 pin as ENET_1G_TX_DATA00
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_09_ENET_1G_TX_DATA00, 0);

   //Set GPIO_DISP_B1_09 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_09_ENET_1G_TX_DATA00,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_10 pin as ENET_1G_TX_EN
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_10_ENET_1G_TX_EN, 0);

   //Set GPIO_DISP_B1_10 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_10_ENET_1G_TX_EN,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));

   //Configure GPIO_DISP_B1_11 pin as ENET_1G_TX_CLK
   IOMUXC_SetPinMux(IOMUXC_GPIO_DISP_B1_11_ENET_1G_TX_CLK_IO, 0);

   //Set GPIO_DISP_B1_11 pad properties
   IOMUXC_SetPinConfig(IOMUXC_GPIO_DISP_B1_11_ENET_1G_TX_CLK_IO,
      IOMUXC_SW_PAD_CTL_PAD_DWP_LOCK(0) |
      IOMUXC_SW_PAD_CTL_PAD_DWP(0) |
      IOMUXC_SW_PAD_CTL_PAD_ODE(0) |
      IOMUXC_SW_PAD_CTL_PAD_PULL(3) |
      IOMUXC_SW_PAD_CTL_PAD_PDRV(0));
}


/**
 * @brief KSZ9563 custom configuration
 * @param[in] interface Underlying network interface
 **/

void ksz9563InitHook(NetInterface *interface)
{
#if 1
   //Force the RGMII bus to operate at 1000 Mb/s in full duplex mode
   ksz9563WriteSwitchReg8(interface, KSZ9563_PORT3_XMII_CTRL1,
      KSZ9563_PORTn_XMII_CTRL1_SPEED_1000 |
      KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_IG |
      KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_EG |
      KSZ9563_PORTn_XMII_CTRL1_IF_TYPE_RGMII);

   ksz9563WriteSwitchReg8(interface, KSZ9563_PORT3_XMII_CTRL0,
      KSZ9563_PORTn_XMII_CTRL0_DUPLEX);
#else
   //Force the RGMII bus to operate at 100 Mb/s in full duplex mode
   ksz9563WriteSwitchReg8(interface, KSZ9563_PORT3_XMII_CTRL1,
      KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_IG |
      KSZ9563_PORTn_XMII_CTRL1_RGMII_ID_EG |
      KSZ9563_PORTn_XMII_CTRL1_IF_TYPE_RGMII);

   ksz9563WriteSwitchReg8(interface, KSZ9563_PORT3_XMII_CTRL0,
      KSZ9563_PORTn_XMII_CTRL0_DUPLEX |
      KSZ9563_PORTn_XMII_CTRL0_SPEED_10_100);
#endif
   //The ports are forced to 10/100 Mbit/s operation only
   ksz9563WritePhyReg(interface, KSZ9563_PORT1, KSZ9563_GBCR, 0);
   ksz9563WritePhyReg(interface, KSZ9563_PORT2, KSZ9563_GBCR, 0);

}


/**
 * @brief Network initialization
 **/

void netUserInit(void)
{
   error_t error;
   uint_t i;

   MacAddr macAddr;
#if (APP_USE_DHCP_CLIENT == DISABLED)
   Ipv4Addr ipv4Addr;
#endif
#if (APP_USE_SLAAC == DISABLED)
   Ipv6Addr ipv6Addr;
#endif

   //Configure debug UART
   debugInit(115200);

   //Start-up message
   TRACE_INFO("\r\n");
   TRACE_INFO("***********************************\r\n");
   TRACE_INFO("*** CycloneTCP HTTP Server Demo ***\r\n");
   TRACE_INFO("***********************************\r\n");
   TRACE_INFO("Copyright: 2010-2023 Oryx Embedded SARL\r\n");
   TRACE_INFO("Compiled: %s %s\r\n", __DATE__, __TIME__);
   TRACE_INFO("Target: i.MX RT1176\r\n");
   TRACE_INFO("\r\n");

   //Initialize hardware cryptographic accelerator
   error = mimxrt1170CryptoInit();
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize hardware crypto accelerator!\r\n");
   }

   //Generate a random seed
   error = trngGetRandomData(seed, sizeof(seed));
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to generate random data!\r\n");
   }

   //PRNG initialization
   error = yarrowInit(&yarrowContext);
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize PRNG!\r\n");
   }

   //Properly seed the PRNG
   error = yarrowSeed(&yarrowContext, seed, sizeof(seed));
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to seed PRNG!\r\n");
   }

#if (SNMP_AGENT_SUPPORT == ENABLED)
   //SNMPv2-MIB initialization
   error = snmpMibInit();
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize SNMPv2-MIB!\r\n");
   }

   //BRIDGE-MIB initialization
   error = bridgeMibInit();
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize BRIDGE-MIB!\r\n");
   }

   //RSTP-MIB initialization
   error = rstpMibInit();
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize RSTP-MIB!\r\n");
   }

   //TCP/IP stack initialization
   error = netInit();
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize TCP/IP stack!\r\n");
   }
#endif

   //Configure the Ethernet interface
   interface = &netInterface[0];

   //Set interface name
   netSetInterfaceName(interface, APP_NAME);
   //Set host name
   netSetHostname(interface, APP_HOST_NAME);
   //Set host MAC address
   macStringToAddr(APP_MAC_ADDR, &macAddr);
   netSetMacAddr(interface, &macAddr);
   //Attach the MAC driver (ENET_1G instance)
   netSetDriver(interface, &mimxrt1170Eth2Driver);
   //Attach the switch driver
   netSetSwitchDriver(interface, &ksz9563SwitchDriver);
   //Attach the SPI driver (management bus)
   netSetSpiDriver(interface, &spiDriver);

   //Initialize network interface
   error = netConfigInterface(interface);
   //Any error to report?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to configure interface %s!\r\n", interface->name);
   }

#if (IPV4_SUPPORT == ENABLED)
#if (APP_USE_DHCP_CLIENT == ENABLED)
   //Get default settings
   dhcpClientGetDefaultSettings(&dhcpClientSettings);
   //Set the network interface to be configured by DHCP
   dhcpClientSettings.interface = interface;
   //Disable rapid commit option
   dhcpClientSettings.rapidCommit = FALSE;

   //DHCP client initialization
   error = dhcpClientInit(&dhcpClientContext, &dhcpClientSettings);
   //Failed to initialize DHCP client?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize DHCP client!\r\n");
   }

   //Start DHCP client
   error = dhcpClientStart(&dhcpClientContext);
   //Failed to start DHCP client?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to start DHCP client!\r\n");
   }
#else
   //Set IPv4 host address
   ipv4StringToAddr(APP_IPV4_HOST_ADDR, &ipv4Addr);
   ipv4SetHostAddr(interface, ipv4Addr);

   //Set subnet mask
   ipv4StringToAddr(APP_IPV4_SUBNET_MASK, &ipv4Addr);
   ipv4SetSubnetMask(interface, ipv4Addr);

   //Set default gateway
   ipv4StringToAddr(APP_IPV4_DEFAULT_GATEWAY, &ipv4Addr);
   ipv4SetDefaultGateway(interface, ipv4Addr);

   //Set primary and secondary DNS servers
   ipv4StringToAddr(APP_IPV4_PRIMARY_DNS, &ipv4Addr);
   ipv4SetDnsServer(interface, 0, ipv4Addr);
   ipv4StringToAddr(APP_IPV4_SECONDARY_DNS, &ipv4Addr);
   ipv4SetDnsServer(interface, 1, ipv4Addr);
#endif
#endif

#if (IPV6_SUPPORT == ENABLED)
#if (APP_USE_SLAAC == ENABLED)
   //Get default settings
   slaacGetDefaultSettings(&slaacSettings);
   //Set the network interface to be configured
   slaacSettings.interface = interface;

   //SLAAC initialization
   error = slaacInit(&slaacContext, &slaacSettings);
   //Failed to initialize SLAAC?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize SLAAC!\r\n");
   }

   //Start IPv6 address autoconfiguration process
   error = slaacStart(&slaacContext);
   //Failed to start SLAAC process?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to start SLAAC!\r\n");
   }
#else
   //Set link-local address
   ipv6StringToAddr(APP_IPV6_LINK_LOCAL_ADDR, &ipv6Addr);
   ipv6SetLinkLocalAddr(interface, &ipv6Addr);
#endif
#endif

#if(RSTP_SUPPORT == ENABLED)
   //Get default settings
   rstpGetDefaultSettings(&rstpBridgeSettings);
   //Underlying network interface
   rstpBridgeSettings.interface = &netInterface[0];
   //Bridge ports
   rstpBridgeSettings.numPorts = RSTP_BRIDGE_NUM_PORTS;
   rstpBridgeSettings.ports = rstpBridgePorts;

   //RSTP bridge initialization
   error = rstpInit(&rstpBridgeContext, &rstpBridgeSettings);
   //Failed to initialize RSTP bridge?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize RSTP bridge!\r\n");
   }

   //Configure bridge ports
   for(i = 1; i <= rstpBridgeContext.numPorts; i++)
   {
      rstpSetAdminPointToPointMac(&rstpBridgeContext, i, RSTP_ADMIN_P2P_MAC_FORCE_TRUE);
      rstpSetAutoEdgePort(&rstpBridgeContext, i, TRUE);
      rstpSetAdminPortState(&rstpBridgeContext, i, TRUE);
   }

   //Start RSTP bridge
   error = rstpStart(&rstpBridgeContext);
   //Failed to start RSTP bridge?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to start RSTP bridge!\r\n");
   }
#endif

#if (HTTP_SERVER_SUPPORT == ENABLED)
   //TLS session cache initialization
   tlsCache = tlsInitCache(8);
   //Any error to report?
   if(tlsCache == NULL)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize TLS session cache!\r\n");
   }

   //Get default settings
   httpServerGetDefaultSettings(&httpServerSettings);
   //Bind HTTP server to the desired interface
   httpServerSettings.interface = &netInterface[0];
   //Listen to port 443
   httpServerSettings.port = HTTPS_PORT;
   //Maximum length of the pending connection queue
   httpServerSettings.backlog = 2;
   //Client connections
   httpServerSettings.maxConnections = APP_HTTP_MAX_CONNECTIONS;
   httpServerSettings.connections = httpConnections;
   //Specify the server's root directory
   strcpy(httpServerSettings.rootDirectory, "/www/");
   //Set default home page
   strcpy(httpServerSettings.defaultDocument, "index.shtm");
   //Callback functions
   httpServerSettings.tlsInitCallback = httpServerTlsInitCallback;
   httpServerSettings.cgiCallback = httpServerCgiCallback;
   httpServerSettings.requestCallback = httpServerRequestCallback;
   httpServerSettings.uriNotFoundCallback = httpServerUriNotFoundCallback;

   //HTTP server initialization
   error = httpServerInit(&httpServerContext, &httpServerSettings);
   //Failed to initialize HTTP server?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize HTTP server!\r\n");
   }

   //Start HTTP server
   error = httpServerStart(&httpServerContext);
   //Failed to start HTTP server?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to start HTTP server!\r\n");
   }
#endif

#if(SNMP_AGENT_SUPPORT == ENABLED)
   //Attach the RSTP bridge context to the BRIDGE-MIB and RSTP-MIB
   bridgeMibSetRstpBridgeContext(&rstpBridgeContext);
   rstpMibSetRstpBridgeContext(&rstpBridgeContext);

   //Get default settings
   snmpAgentGetDefaultSettings(&snmpAgentSettings);
   //Minimum version accepted by the SNMP agent
   snmpAgentSettings.versionMin = SNMP_VERSION_1;
   //Maximum version accepted by the SNMP agent
   snmpAgentSettings.versionMax = SNMP_VERSION_2C;
   //SNMP port number
   snmpAgentSettings.port = SNMP_PORT;
   //SNMP trap port number
   snmpAgentSettings.trapPort = SNMP_TRAP_PORT;

   //SNMP agent initialization
   error = snmpAgentInit(&snmpAgentContext, &snmpAgentSettings);
   //Failed to initialize SNMP agent?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to initialize SNMP agent!\r\n");
   }

   //Load SNMPv2-MIB
   snmpAgentLoadMib(&snmpAgentContext, &snmpMibModule);
   //Load BRIDGE-MIB
   snmpAgentLoadMib(&snmpAgentContext, &bridgeMibModule);
   //Load RSTP-MIB
   snmpAgentLoadMib(&snmpAgentContext, &rstpMibModule);

   //Set read-only community string
   snmpAgentCreateCommunity(&snmpAgentContext, "public",
      SNMP_ACCESS_READ_ONLY);

   //Set read-write community string
   snmpAgentCreateCommunity(&snmpAgentContext, "private",
      SNMP_ACCESS_READ_WRITE);

   //Start SNMP agent
   error = snmpAgentStart(&snmpAgentContext);
   //Failed to start SNMP agent?
   if(error)
   {
      //Debug message
      TRACE_ERROR("Failed to start SNMP agent!\r\n");
   }
#endif

   //Start TLS 1.3 PSK server
   //tlsServerDemoInit();
}
