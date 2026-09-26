#ifndef V2C_NODE_CONFIG_H
#define V2C_NODE_CONFIG_H

#ifndef V2C_NODE_INDEX
#define V2C_NODE_INDEX 2
#endif

#define V2C_RBS_ID       0x03
#define V2C_BROADCAST_ID 0xFF

#if V2C_NODE_INDEX == 1
  #define V2C_PAIR_INDEX 1
  #define V2C_SU_ID      0x01
  #define V2C_DU_ID      0x02
#elif V2C_NODE_INDEX == 2
  #define V2C_PAIR_INDEX 2
  #define V2C_SU_ID      0x04
  #define V2C_DU_ID      0x05
#else
  #error "V2C_NODE_INDEX phai la 1 hoac 2"
#endif

#endif
