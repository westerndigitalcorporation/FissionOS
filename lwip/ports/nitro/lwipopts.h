#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define SYS_LIGHTWEIGHT_PROT                    1
#define NO_SYS                                  0
#define MEMCPY(dst,src,len)                     memcpy(dst,src,len)
#define SMEMCPY(dst,src,len)                    memcpy(dst,src,len)
#define MEM_LIBC_MALLOC                         1
#define MEMP_MEM_MALLOC                         0
#define MEM_ALIGNMENT                           4
#define MEM_SIZE                                (128 * 1024)
#define MEMP_OVERFLOW_CHECK                     0
#define MEMP_SANITY_CHECK                       0
#define MEM_USE_POOLS                           0
#define MEM_USE_POOLS_TRY_BIGGER_POOL           0
#define MEMP_USE_CUSTOM_POOLS                   0
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT  0
#define MEMP_NUM_PBUF                           16
#define MEMP_NUM_RAW_PCB                        4
#define MEMP_NUM_UDP_PCB                        4
#define MEMP_NUM_TCP_PCB                        8
#define MEMP_NUM_TCP_PCB_LISTEN                 8
#define MEMP_NUM_TCP_SEG                        16
#define MEMP_NUM_REASSDATA                      5
#define MEMP_NUM_ARP_QUEUE                      8
#define MEMP_NUM_IGMP_GROUP                     0
#define MEMP_NUM_SYS_TIMEOUT                    16
#define MEMP_NUM_NETBUF                         16
#define MEMP_NUM_NETCONN                        8
#define MEMP_NUM_TCPIP_MSG_API                  1
#define MEMP_NUM_TCPIP_MSG_INPKT                32

#define PBUF_POOL_SIZE                          64

#define LWIP_ARP                                1
#define ARP_TABLE_SIZE                          10
#define ARP_QUEUEING                            1

#define ETHARP_TRUST_IP_MAC                     1
#define ETHARP_SUPPORT_VLAN                     0

#define IP_FORWARD                              0
#define IP_OPTIONS_ALLOWED                      1
#define IP_REASSEMBLY                           1
#define IP_REASS_MAXAGE                         3
#define IP_REASS_MAX_PBUFS                      10
#define IP_FRAG_USES_STATIC_BUF                 1
#if IP_FRAG_USES_STATIC_BUF && !defined(IP_FRAG_MAX_MTU)
#define IP_FRAG_MAX_MTU                         1500
#endif
#define IP_DEFAULT_TTL                          255
#define IP_SOF_BROADCAST                        0
#define IP_SOF_BROADCAST_RECV                   0

#define LWIP_ICMP                               1
#define ICMP_TTL                                (IP_DEFAULT_TTL)
#define LWIP_BROADCAST_PING                     0
#define LWIP_MULTICAST_PING                     0

#define LWIP_RAW                                0
#define RAW_TTL                                 (IP_DEFAULT_TTL)

#define LWIP_DHCP                               1
#define DHCP_DOES_ARP_CHECK                     ((LWIP_DHCP) && (LWIP_ARP))
#define DHCP_CREATE_RAND_XID                    1
#define LWIP_AUTOIP                             1
#define LWIP_DHCP_AUTOIP_COOP                   1
#define LWIP_DHCP_AUTOIP_COOP_TRIES             2

#define LWIP_IGMP                               0

#define LWIP_DNS                                0
#define DNS_TABLE_SIZE                          4
#define DNS_MAX_NAME_LENGTH                     256
#define DNS_MAX_SERVERS                         2
#define DNS_DOES_NAME_CHECK                     1
#define DNS_USES_STATIC_BUF                     1
#define DNS_MSG_SIZE                            512
#define DNS_LOCAL_HOSTLIST                      0
#define DNS_LOCAL_HOSTLIST_IS_DYNAMIC           0

#define LWIP_UDP                                1
#define LWIP_UDPLITE                            0
#define UDP_TTL                                 (IP_DEFAULT_TTL)
#define LWIP_NETBUF_RECVINFO                    0

#define LWIP_TCP                                1
#define TCP_TTL                                 (IP_DEFAULT_TTL)
#define TCP_WND                                 (12 * TCP_MSS)
#define TCP_MAXRTX                              12
#define TCP_SYNMAXRTX                           6
#define TCP_QUEUE_OOSEQ                         (LWIP_TCP)
#define TCP_MSS                                 1460
#define TCP_CALCULATE_EFF_SEND_MSS              1
#define TCP_SND_BUF                             (2 * TCP_MSS)
#define TCP_SND_QUEUELEN                        (4 * (TCP_SND_BUF) / (TCP_MSS))
#define TCP_SNDLOWAT                            ((TCP_SND_BUF) / 2)
#define TCP_LISTEN_BACKLOG                      0
#define TCP_DEFAULT_LISTEN_BACKLOG              0xff
#define LWIP_TCP_TIMESTAMPS                     0
#define TCP_WND_UPDATE_THRESHOLD                (TCP_WND / 4)

#define PBUF_LINK_HLEN                          14
#define PBUF_POOL_BUFSIZE                       LWIP_MEM_ALIGN_SIZE(TCP_MSS + 40 + PBUF_LINK_HLEN)

#define LWIP_NETIF_HOSTNAME                     1
#define LWIP_NETIF_API                          0
#define LWIP_NETIF_STATUS_CALLBACK              0
#define LWIP_NETIF_LINK_CALLBACK                0
#define LWIP_NETIF_HWADDRHINT                   0
#define LWIP_NETIF_LOOPBACK                     0
#define LWIP_LOOPBACK_MAX_PBUFS                 0
#define LWIP_NETIF_LOOPBACK_MULTITHREADING      (!NO_SYS)
#define LWIP_NETIF_TX_SINGLE_PBUF               0
#define LWIP_HAVE_LOOPIF                        0
#define LWIP_HAVE_SLIPIF                        0

#define TCPIP_THREAD_NAME                       "tcpip_thread"
#define TCPIP_THREAD_STACKSIZE                  (4096)
#define TCPIP_THREAD_PRIO                       0
#define TCPIP_MBOX_SIZE                         0

#define LWIP_TCPIP_CORE_LOCKING                 0
#define LWIP_NETCONN                            1
#define LWIP_SOCKET                             0
#define LWIP_COMPAT_SOCKETS                     0
#define LWIP_POSIX_SOCKETS_IO_NAMES             0
#define LWIP_TCP_KEEPALIVE                      1
#define LWIP_SO_RCVTIMEO                        1
#define LWIP_SO_RCVBUF                          0
#define RECV_BUFSIZE_DEFAULT                    INT_MAX
#define SO_REUSE                                0

#define LWIP_STATS                              1
#if LWIP_STATS
#define LWIP_STATS_DISPLAY                      1
#define LINK_STATS                              0
#define ETHARP_STATS                            0
#define IP_STATS                                0
#define IPFRAG_STATS                            0
#define ICMP_STATS                              0
#define IGMP_STATS                              0
#define UDP_STATS                               1
#define TCP_STATS                               1
#define MEM_STATS                               0
#define MEMP_STATS                              1
#define SYS_STATS                               0
#endif /* LWIP_STATS */

#define CHECKSUM_GEN_IP                         1
#define CHECKSUM_GEN_UDP                        1
#define CHECKSUM_GEN_TCP                        1
#define CHECKSUM_CHECK_IP                       1
#define CHECKSUM_CHECK_UDP                      1
#define CHECKSUM_CHECK_TCP                      1

#define LWIP_TIMEVAL_PRIVATE                    1

#define LWIP_COMPAT_MUTEX                       1

#define LWIP_PLATFORM_BYTESWAP                  1
#define LWIP_PLATFORM_HTONS(x)                  ( (((u16_t)(x))>>8) | (((x)&0xFF)<<8) )
#define LWIP_PLATFORM_HTONL(x)                  ( (((u32_t)(x))>>24) | (((x)&0xFF0000)>>8) | \
                                                  (((x)&0xFF00)<<8) | (((x)&0xFF)<<24) )

#endif /* __LWIPOPTS_H__ */


