#! /bin/sh
### BEGIN INIT INFO
# Provides:          rtnet
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: rtnet
# Description:       rtnet
### END INIT INFO

RTNET_BIN_DIR=/usr/xenomai/sbin
export PATH=$PATH:$RTNET_BIN_DIR

#ETH_DRV=igb
ETH_DRV=e1000e
RT_ETH_DRV=rt_$ETH_DRV

#REBIND_RT_NICS=$(ethtool -i $ETH_IFACE | grep bus-info | cut -d ' ' -f 2)
REBIND_RT_NICS="0000:03:00.0"

. /lib/lsb/init-functions

case "$1" in
  start)
    log_action_msg "Configure rtnet" 
    chgrp xenomai /dev/rtp*
    chmod g+rw /dev/rtp*
    /bin/echo "probing modules" 
    modprobe $RT_ETH_DRV 
    modprobe rtpacket
    echo "Rebind pci " $REBIND_RT_NICS
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$ETH_DRV/unbind
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$RT_ETH_DRV/bind
    /bin/echo "ifup rteth0" 
    rtifconfig rteth0 up
    log_end_msg $?
  ;;

  stop)
    log_action_msg "DeConfigure rtnet"
    /bin/echo "ifdown rteth0" 
    rtifconfig rteth0 down
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$RT_ETH_DRV/unbind
    /bin/echo $REBIND_RT_NICS > /sys/bus/pci/drivers/$ETH_DRV/bind
    log_end_msg $?
  ;;

  restart|force-reload)
    $0 stop
    $0 start
  ;;
  
  status)
    lsmod | grep rt
  ;;
  
  *)
    echo "Usage: $0 {start|stop|restart|status}"
    exit 1
  ;;
esac

exit
