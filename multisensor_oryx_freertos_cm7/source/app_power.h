
#ifndef _APP_POWER_H
#define _APP_POWER_H


// power flag bitfield definitions here
#define	POWERFLAG_SIDEA_EFUSE_FAULT			0x00000001	//Side A – e-Fuse Fault
#define	POWERFLAG_SIDEB_EFUSE_FAULT			0x00000002	//Side B – e-Fuse Fault
#define	POWERFLAG_SIDEA_EFUSE_SHUTDOWN		0x00000004	//Side A – e-Fuse is shutdown
#define	POWERFLAG_SIDEB_EFUSE_SHUTDOWN		0x00000008	//Side B – e-Fuse is shutdown
#define	POWERFLAG_SIDEA_POWERSOURCE			0x00000010	//Power Source is Side A
#define	POWERFLAG_SIDEB_POWERSOURCE			0x00000020	//Power Source is Side B



void power_task(void *pvParameters);


#endif
